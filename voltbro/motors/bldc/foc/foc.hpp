#pragma once
#include <cstdint>
#include "voltbro/encoders/generic.h"
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>
#include <utility>

#include "../bldc.h"
#include "voltbro/math/regulators/pid.hpp"
#include "voltbro/math/dsp/low_pass_filter.hpp"

#define USE_CALIBRATION_ARRAY
constexpr size_t CALIBRATION_BUFF_SIZE = 2048;
using __non_const_calib_array_t = std::array<int, CALIBRATION_BUFF_SIZE>;
using calibration_array_t = const __non_const_calib_array_t;

struct __attribute__((packed)) CalibrationData {
    static constexpr uint32_t TYPE_ID = 0x89ABCDEF;
    bool was_calibrated;
    bool is_encoder_inverted;
    uint16_t ppair_counter;
    int meas_elec_offset;
    uint32_t type_id = 0;
#ifdef USE_CALIBRATION_ARRAY
    __non_const_calib_array_t calibration_array;
#endif

    CalibrationData() {
        reset();
        type_id = 0;  // to distinguish between uninitialized and reset data
    }

    void reset() {
        type_id = TYPE_ID;
        was_calibrated = false;
        is_encoder_inverted = false;
        ppair_counter = 0;
        meas_elec_offset = 0;
#ifdef USE_CALIBRATION_ARRAY
        calibration_array.fill(0);
#endif
    }
};

struct FOCTarget {
    float torque = 0.0f;
    float angle = 0.0f;
    float velocity = 0.0f;
    float angle_kp = 0.0f;
    float velocity_kp = 0.0f;
};

struct FiltersConfig {
    float expected_a;
    float g1;
    float g2;
    float g3;
    float I_lpf_coefficient;
    float velocity_lpf_coefficient = 1.0f;
    float i_q_slew_rate = 0.0f;
};

/**
 * Field oriented control.
 */
class FOC: public BLDCController  {
protected:
    float T;
    float raw_elec_angle = 0;
    float elec_angle = 0;
    float I_Q = 0;
    calibration_array_t* lookup_table = nullptr;
    GenericEncoder& encoder;
    const FiltersConfig filters_config;
    LowPassFilter velocity_lpf;
    float control_velocity = 0.0f;
    bool is_control_velocity_initialized = false;
    float i_q_set_slewed = 0.0f;
    FOCTarget foc_target;
    PIDRegulator q_reg;
    PIDRegulator d_reg;
    PIDRegulator control_reg = PIDRegulator();

    void apply_kalman();
    void update_angle();
    virtual void update_shaft_angle();

    void set_windings_calibration(float current_angle);
public:
    virtual void calibrate(CalibrationData& calibration_data, std::byte* additional_buffer, size_t buffer_size);
    void apply_calibration(CalibrationData& calibration_data) {
        // Replace config parameters with the ones loaded from EEPROM
        // Very ugly hack, but it will work for now
        const_cast<int&>(encoder.electric_offset) = calibration_data.meas_elec_offset;
#ifdef USE_CALIBRATION_ARRAY
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
        // TODO: verify that this is safe, but it works so probably fine?
        lookup_table = &(calibration_data.calibration_array);
        #pragma GCC diagnostic pop
#endif
        const_cast<bool&>(encoder.is_inverted) = calibration_data.is_encoder_inverted;
    }

    FOC(
        float T,
        FiltersConfig&& filters_config_,
        PIDConfig&& q_config,
        PIDConfig&& d_config,
        const DriveLimits& drive_limits,
        const DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        GenericEncoder& encoder,
        BaseInverter& inverter
    ):
        BLDCController(
            drive_limits,
            drive_info,
            htim,
            inverter
        ),
        T(T),
        encoder(encoder),
        filters_config(filters_config_),
        velocity_lpf(filters_config.velocity_lpf_coefficient),
        q_reg(std::move(q_config)),
        d_reg(std::move(d_config))
        {}

    float get_electric_angle() {
        return elec_angle;
    }
    float get_working_current() {
        return I_Q;
    }

    bool set_foc_point(FOCTarget&& target) {
        if (
            !is_torque_target_valid(target.torque) ||
            !is_angle_target_valid(target.angle) ||
            !is_velocity_target_valid(target.velocity)
        ) {
            return false;
        }
        point_type = SetPointType::UNIVERSAL;
        foc_target = std::move(target);
        return true;
    }
    void update_q_config(PIDConfig&& new_config) {
        q_reg.update_config(std::move(new_config));
    }
    void update_d_config(PIDConfig&& new_config) {
        d_reg.update_config(std::move(new_config));
    }
    void update_control_config(PIDConfig&& new_config) {
        control_reg.update_config(std::move(new_config));
    }
    const GenericEncoder& get_encoder() const {
        return encoder;
    }

    HAL_StatusTypeDef init() override {
        HAL_StatusTypeDef result = BLDCController::init();
        if (result != HAL_OK) {
            return result;
        }

        return encoder.init();
    }
    void update() override;
    virtual void update_sensors();
};

#endif
#endif
