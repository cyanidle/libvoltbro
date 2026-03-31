#pragma once
#include <cstdint>
#include "voltbro/utils.hpp"
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_spi.h"

#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_SPI_MODULE_ENABLED) && defined(HAL_I2C_MODULE_ENABLED)

#include "../foc/foc.hpp"
#include <voltbro/encoders/ASxxxx/AS5047P.hpp>
#include <voltbro/devices/stspin32g4.hpp>
#include <voltbro/math/dsp/low_pass_filter.hpp>
#include <voltbro/eeprom/eeprom.hpp>
#include "voltbro/generics/spi_mixin.hpp"

class VBInverter final: public BaseInverter {
private:
    static constexpr float adc_conversion_factor = 3.3f / (1.0f * 4096.0f);

    const volatile uint32_t __attribute__((aligned(4))) ADC_1_buffer[3] = {};
    ADC_HandleTypeDef* hadc_1;
    const volatile uint32_t __attribute__((aligned(4))) ADC_2_buffer[2] = {};
    ADC_HandleTypeDef* hadc_2;

    float stator_temperature = 0;
    float mcu_temperature = 0;
public:
    VBInverter(ADC_HandleTypeDef* hadc_1, ADC_HandleTypeDef* hadc_2):
        hadc_1(hadc_1), hadc_2(hadc_2) {};

    float get_stator_temperature() const { return stator_temperature; }
    float get_mcu_temperature() const { return mcu_temperature; }

    FORCE_INLINE float read_raw_A() const {
        return (float)ADC_1_buffer[0] * adc_conversion_factor;
    }
    FORCE_INLINE float read_raw_B() const {
        return (float)ADC_2_buffer[0] * adc_conversion_factor;
    }
    FORCE_INLINE float read_raw_V() const {
        return 16.0f * (float)ADC_1_buffer[1] * adc_conversion_factor;
    }
    FORCE_INLINE float read_raw_T() const {
        return (float)ADC_1_buffer[2]*1.1f;
    }

    void start() override{
        if (has_started()) {
            return;
        }
        is_started = true;

        HAL_ADCEx_Calibration_Start(hadc_1, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(hadc_2, ADC_SINGLE_ENDED);
        HAL_ADC_Start_DMA(hadc_1, const_cast<uint32_t *>(ADC_1_buffer), 3);
        HAL_ADC_Start_DMA(hadc_2, const_cast<uint32_t *>(ADC_2_buffer), 2);

        HAL_Delay(100);

        // Record offset;
        int cycles = 100;
        for (int i = 0; i < cycles; i++) {
            HAL_Delay(2);
            I_A_offset += read_raw_A();
            I_B_offset += read_raw_B();
        }
        I_A_offset /= (float)cycles;
        I_B_offset /= (float)cycles;
    }

    void update() override {
        if (!has_started()) {
            return;
        }

        constexpr float shunt_res = 0.003f;
        constexpr float op_amp_gain = 20.0f;
        constexpr float conv_factor = shunt_res * op_amp_gain;

        I_A = (read_raw_A() - I_A_offset ) / conv_factor;
        I_B = (read_raw_B() - I_B_offset ) / conv_factor;
        busV = read_raw_V();
        I_C = -I_A - I_B;
    }

    void update_temperature() {
        const float TS_CAL1_TEMP = 30.0f;
        const float TS_CAL2_TEMP = 130.0f;
        volatile float TS_CAL1 = (float)*(uint16_t*)0x1FFF75A8;
        volatile float TS_CAL2 = (float)*(uint16_t*)0x1FFF75CA;
        mcu_temperature = (TS_CAL2_TEMP - TS_CAL1_TEMP) * (read_raw_T() - TS_CAL1) / ( TS_CAL2 - TS_CAL1 ) + TS_CAL1_TEMP;

        float thermistor = 1.0f / (4095.0f / ADC_2_buffer[1] - 1.0f);
        float steinhart = logf(thermistor);
        steinhart /= 3950.0f;
        steinhart += 1.0f / (25.0f + 273.15f);
        steinhart = 1.0f / steinhart;
        stator_temperature = steinhart - 273.15f;
    }
};

class InductiveSensor: public SPIMixin {
public:
    struct State {
        static constexpr uint32_t TYPE_ID = 0xAAAAAA99;
        uint32_t type_id;
        bool was_programmed;

        void reset() {
            was_programmed = false;
            type_id = TYPE_ID;
        }
    };
protected:
    encoder_data raw_value = 0;
    static const uint32_t read_pos_cmd = 209;

    const uint16_t state_location;

    EEPROM& eeprom;
    bool is_started = false;
    GpioPin spi_cs;
    float current_angle = NAN;
    float current_speed = 0;
    int revolutions = 0;
    LowPassFilter speed_filter = LowPassFilter(0.1f);

    uint32_t transmit_command(uint32_t TxData) {
        spi_cs.reset();

        uint32_t retval = 0;

        LL_SPI_TransmitData16(spi->Instance, (uint16_t)( TxData >> 16 ));
        while(!LL_SPI_IsActiveFlag_RXNE(spi->Instance));

        retval = (uint32_t)LL_SPI_ReceiveData16(spi->Instance) << 16;

        LL_SPI_TransmitData16(spi->Instance, (uint16_t)( TxData ));
        while(!LL_SPI_IsActiveFlag_RXNE(spi->Instance));

        retval += (uint32_t)LL_SPI_ReceiveData16(spi->Instance);

        spi_cs.set();

        return retval;
    }

    void program() {
        InductiveSensor::State state;
        HAL_IMPORTANT(eeprom.read<InductiveSensor::State>(&state, state_location));
        if (state.type_id != InductiveSensor::State::TYPE_ID) {
            state.reset();
        }

        LL_GPIO_SetOutputPin(INDUCT_EN_GPIO_Port, INDUCT_EN_Pin);

        if (state.was_programmed) {
            return;
        }

        //#pragma region ENCODER_PROGRAMMING
        LL_SPI_Enable(SPI3);

        HAL_Delay(20);

        constexpr uint32_t SERV_MODE_CMD = 64318;
        constexpr uint32_t ENABLE_SPI_CMD = 270593;
        constexpr uint32_t WRITE_EEPROM_1_CMD = 8979;
        constexpr uint32_t EXIT_SRV_MODE_CMD = 897;
        constexpr uint32_t PWL_Y0_CMD = 13569;
        constexpr uint32_t PWL_X1_CMD = 4294916353;
        constexpr uint32_t PWL_Y1_CMD = 4294917377;
        constexpr uint32_t WRITE_EEPROM_2_CMD = 14113;

        transmit_command(SERV_MODE_CMD);
        transmit_command(ENABLE_SPI_CMD);
        transmit_command(WRITE_EEPROM_1_CMD);

        HAL_Delay(100);

        transmit_command(EXIT_SRV_MODE_CMD);

        // reboot
        LL_GPIO_ResetOutputPin(INDUCT_EN_GPIO_Port, INDUCT_EN_Pin);
        HAL_Delay(20);
        LL_GPIO_SetOutputPin(INDUCT_EN_GPIO_Port, INDUCT_EN_Pin);
        HAL_Delay(20);

        transmit_command(SERV_MODE_CMD);
        transmit_command(PWL_Y0_CMD);
        transmit_command(PWL_X1_CMD);
        transmit_command(PWL_Y1_CMD);
        transmit_command(WRITE_EEPROM_2_CMD);

        HAL_Delay(100);

        transmit_command(EXIT_SRV_MODE_CMD);

        //#pragma endregion

        state.was_programmed = true;
        HAL_IMPORTANT(eeprom.write<InductiveSensor::State>(&state, state_location));
    }

public:
    InductiveSensor(
        EEPROM& eeprom,
        uint16_t state_location,
        SPI_HandleTypeDef* hspi,
        GpioPin spi_cs
    ):
        SPIMixin(hspi),
        state_location(state_location),
        eeprom(eeprom),
        spi_cs(spi_cs)
        {}

    void init() {
        if (has_started()) {
            return;
        }
        LL_SPI_Enable(spi->Instance);
        program();
        is_started = true;
    }

    bool has_started() const {
        return is_started;
    }

    FORCE_INLINE int get_revolutions() {
        return revolutions;
    }

    FORCE_INLINE float get_angle() {
        return current_angle;
    }

    FORCE_INLINE float get_speed() {
        return current_speed;
    }

    FORCE_INLINE encoder_data get_raw_value() {
        return raw_value;
    }

    void update() {
        static bool induct_comm_phase = 0;

        spi_cs.reset(); // CS low

        if (!induct_comm_phase) {
            // Upper 16 bits

            uint16_t tx_upper = (uint16_t)(read_pos_cmd >> 16);
            raw_value = spi_transmit_command_receive(tx_upper);
            //HAL_SPI_TransmitReceive(hspi, (uint8_t*)&tx_upper, (uint8_t*)&raw_value, 1, 1000);

            float new_angle = pi2 * static_cast<float>(raw_value) / 65535.0f;
            float diff = new_angle - current_angle;
            if (abs(diff) > PI) {
                if (diff < 0) {
                    revolutions += 1;
                    diff += pi2;
                }
                else {
                    revolutions -= 1;
                    diff -= pi2;
                }
            }
            /*
            if (!is_close(dt, 0, 1e-8)) {
                current_speed = speed_filter(current_speed, diff / dt);
            }
            */
            current_angle = new_angle;

            induct_comm_phase = true;
        } else {
            // Lower 16 bits
            //uint16_t tx_lower = (uint16_t)(read_pos_cmd);
            //uint16_t rx_lower = 0;
            //spi_transmit_only(tx_lower);
            //HAL_SPI_TransmitReceive(hspi, (uint8_t*)&tx_lower, (uint8_t*)&rx_lower, 1, 1000);

            spi_cs.set(); // CS high
            induct_comm_phase = false;
        }
    }
};

enum class AngleEncoderType : uint8_t {
    ROTOR,
    SHAFT
};

class VBDrive final: public FOC {
protected:
    static constexpr uint32_t bootstrap_charge_time_ms = 5U;

    AngleEncoderType angle_encoder =  AngleEncoderType::ROTOR;
    STSPIN32G4& gate_driver;
    InductiveSensor& inductive_sensor;
    arm_atomic(uint32_t) bootstrap_charge_deadline_ms = 0;

    FORCE_INLINE bool is_bootstrap_charging(uint32_t now_ms) const {
        return static_cast<int32_t>(bootstrap_charge_deadline_ms - now_ms) > 0;
    }

    FORCE_INLINE void force_bootstrap_charge() {
        DQs[0] = 0;
        DQs[1] = 0;
        DQs[2] = 0;
        set_pwm();
    }

    void update_shaft_angle() override {
        if (angle_encoder == AngleEncoderType::SHAFT) {
            shaft_angle = inductive_sensor.get_revolutions() * pi2 + inductive_sensor.get_angle();
        }
        else {
            FOC::update_shaft_angle();
        }
    }

public:
    VBDrive(
        float T,
        FiltersConfig&& filters_config,
        PIDConfig&& q_config,
        PIDConfig&& d_config,
        const DriveLimits& drive_limits,
        const DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        AS5047P& encoder,
        VBInverter& inverter,
        STSPIN32G4& gate_driver,
        InductiveSensor& inductive_sensor,
        AngleEncoderType angle_encoder
    ):
        FOC(
            T,
            std::move(filters_config),
            std::move(q_config),
            std::move(d_config),
            drive_limits,
            drive_info,
            htim,
            encoder,
            inverter
        ),
        angle_encoder(angle_encoder),
        gate_driver(gate_driver),
        inductive_sensor(inductive_sensor)
        {}

        void set_current_regulator_params(float kp, float ki) {
            q_reg.update_config(kp, ki, 0);
            d_reg.update_config(kp, ki, 0);
        }

        void update() override {
            static uint32_t iteration = 0;
            static uint32_t inductive_update_counter = 0;
            // Per datasheet, maximum "temporal frequency" of this encoder is ~4.63 kHz
            // Because datasheet specifies the "Position DSP Update Rate tPER" as 216 µs.
            EACH_N(iteration, inductive_update_counter, 9, {
                inductive_sensor.update();
            })
            iteration += 1;

            if (!_is_on) {
                FOC::update_sensors();
                return;
            }

            const uint32_t now_ms = HAL_GetTick();
            if (is_bootstrap_charging(now_ms)) {
                FOC::update_sensors();
                force_bootstrap_charge();
                return;
            }

            FOC::update();
        }

        // for logging
        encoder_data get_rotor_encoder_value() {
            return encoder.get_value();
        }
        encoder_data get_shaft_encoder_value() {
            return inductive_sensor.get_raw_value();
        }

        HAL_StatusTypeDef init() override {
            HAL_StatusTypeDef result = FOC::init();
            if (result != HAL_OK) {
                return result;
            }
            result = HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1); // AN-phase
            if (result != HAL_OK) {
                return result;
            }
            result = HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2); // BN-phase
            if (result != HAL_OK) {
                return result;
            }
            result = HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3); // CN-phase

            inductive_sensor.init();

            return result;
        }

        HAL_StatusTypeDef stop() override {
            _is_on = false;
            bootstrap_charge_deadline_ms = 0;
            __HAL_TIM_MOE_DISABLE(htim);

            HAL_StatusTypeDef result = gate_driver.request_standby();
            if (result != HAL_OK) {
                return result;
            }
            return HAL_OK;
        }

        HAL_StatusTypeDef start() override {
            bootstrap_charge_deadline_ms = HAL_GetTick() + bootstrap_charge_time_ms;

            HAL_StatusTypeDef result = gate_driver.wake();
            if (result != HAL_OK) {
                bootstrap_charge_deadline_ms = 0;
                return result;
            }

            __HAL_TIM_MOE_ENABLE(htim);
            force_bootstrap_charge();

            result = gate_driver.clear_faults();
            if (result != HAL_OK) {
                __HAL_TIM_MOE_DISABLE(htim);
                bootstrap_charge_deadline_ms = 0;
                return result;
            }

            quit_stall();
            _is_on = true;
            return HAL_OK;
        }

        FORCE_INLINE void set_pwm() override {
            // limiting duty cycle to give ADC time to sample current reading
            // RM0440 21.4.12
            for (size_t index = 0; index < 3; index++) {
                if (DQs[index] > 1760) {
                    DQs[index] = 1760;
                }
            }

            htim->Instance->CCR1 = DQs[0];
            htim->Instance->CCR2 = DQs[1];
            htim->Instance->CCR3 = DQs[2];
        }
};

#endif
#endif
