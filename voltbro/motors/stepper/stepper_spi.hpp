#pragma once
#if defined(STM32G4) || defined(STM32_G)

#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_SPI_MODULE_ENABLED)

#include <bit>
#include <utility>

#include "stepper_base.hpp"

struct StepperSPIConfig {
    const GpioPin direction;
    const GpioPin sd_mode;
    const GpioPin spi_mode;

    const GpioPin cfg4;
    const GpioPin cfg5;
    const GpioPin cfg6;

    GpioPin spi_ss;
    SPI_HandleTypeDef* const spi;

    const pwm_channel step_channel;
    TIM_HandleTypeDef* const timer;
};

#define check_status(command) status = command; if (status != HAL_OK) { return status; }

class StepperMotorSPI : public StepperBase {
protected:
    const StepperSPIConfig config;
    arm_atomic(bool) _is_on = false;
    arm_atomic(int32_t) position;
public:
    StepperMotorSPI(const StepperSPIConfig&& driver, GpioPin&& enn_pin):
        StepperBase(std::forward<GpioPin&&>(enn_pin)),
        config(std::move(driver))
    {};

    int32_t get_position() {
        return position;
    }

    HAL_StatusTypeDef send_recieve_data(uint8_t address, uint32_t in, uint32_t* out) {
        uint8_t tx_buffer[4];
        uint8_t rx_buffer[4];
        tx_buffer[0] = (in >> 24) & 0xFF;
        tx_buffer[1] = (in >> 16) & 0xFF;
        tx_buffer[2] = (in >> 8) & 0xFF;
        tx_buffer[3] = in & 0xFF;

        HAL_StatusTypeDef status;

        config.spi_ss.reset();
        delay_cpu_cycles(CYCLES_100NS_160Mhz * 10 * 11);
        check_status(HAL_SPI_Transmit(config.spi, &address, 1, HAL_MAX_DELAY))
        check_status(HAL_SPI_TransmitReceive(config.spi, tx_buffer, rx_buffer, 4, HAL_MAX_DELAY))
        config.spi_ss.set();

        uint32_t received_data = 0;
        received_data |= (uint32_t)rx_buffer[0] << 24;
        received_data |= (uint32_t)rx_buffer[1] << 16;
        received_data |= (uint32_t)rx_buffer[2] << 8;
        received_data |= (uint32_t)rx_buffer[3];
        *out = received_data;

        return HAL_OK;
    }

    void update_position(bool skip_first=true) {
        uint32_t raw_position = 0;
        if (!skip_first) {
            send_recieve_data(0x21, 0x00000000, &raw_position);
        }
        send_recieve_data(0x21, 0x00000000, &raw_position);
        position = std::bit_cast<int32_t>(raw_position);
    }

    virtual HAL_StatusTypeDef send_config() {
        uint32_t recv;
        HAL_StatusTypeDef status;
        check_status(send_recieve_data(0x80, 0x00000000, &recv))
        check_status(send_recieve_data(0xEC, 0x000101D5, &recv))
        check_status(send_recieve_data(0x90, 0x00070603, &recv))
        check_status(send_recieve_data(0x91, 0x0000000A, &recv))

        check_status(send_recieve_data(0xF0, 0x00000000, &recv))

        check_status(send_recieve_data(0xA4, 0x000003E8, &recv))
        check_status(send_recieve_data(0xA5, 0x000186A0, &recv))
        check_status(send_recieve_data(0xA6, 0x0000C350, &recv))
        check_status(send_recieve_data(0xA7, 0x000186A0, &recv))
        check_status(send_recieve_data(0xAA, 0x00000578, &recv))
        check_status(send_recieve_data(0xAB, 0x0000000A, &recv))
        check_status(send_recieve_data(0xA0, 0x00000000, &recv))

        check_status(send_recieve_data(0xA1, 0x00000000, &recv))

        check_status(send_recieve_data(0x21, 0x00000000, &recv));

        return HAL_OK;
    }

    HAL_StatusTypeDef init() override {
        config.sd_mode.reset();
        config.spi_mode.set();
        HAL_Delay(150);
        return send_config();
    }

    virtual void set_target(int32_t value) {
        uint32_t recv;
        send_recieve_data(0xAD, std::bit_cast<uint32_t>(value), &recv);
    }

    void update() override {
        update_position();
    }
};

#endif
#endif
