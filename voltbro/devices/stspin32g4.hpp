#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_conf.h"
#ifdef HAL_I2C_MODULE_ENABLED

#include <cstdint>

#include "voltbro/devices/gpio.hpp"

class STSPIN32G4 {
public:
    enum class Register : uint16_t {
        CLEAR = 0x09,
        STANDBY = 0x0A,
        LOCK = 0x0B,
    };

private:
    static constexpr uint16_t device_address = 0x47U << 1U;
    static constexpr uint32_t register_timeout_ms = 20U;
    static constexpr uint8_t lock_unlock_value = 0x0FU;
    static constexpr uint8_t lock_locked_value = 0x00U;
    static constexpr uint8_t command_execute_value = 0xFFU;
    static constexpr uint8_t standby_bit = 0x01U;

    I2C_HandleTypeDef* const hi2c;
    const GpioPin wake_pin;

    static constexpr uint16_t to_address(Register reg) {
        return static_cast<uint16_t>(reg);
    }

    HAL_StatusTypeDef is_device_ready(uint32_t trials = 2U, uint32_t timeout_ms = 10U) const;
    HAL_StatusTypeDef write_register(Register reg, uint8_t value) const;
    HAL_StatusTypeDef write_lock_register(uint8_t value) const;

public:
    STSPIN32G4(I2C_HandleTypeDef* hi2c, GpioPin wake_pin)
        : hi2c(hi2c), wake_pin(wake_pin) {}

    HAL_StatusTypeDef clear_faults() const;
    HAL_StatusTypeDef wake(uint32_t timeout_ms = 50U) const;
    HAL_StatusTypeDef request_standby() const;
};

#endif
#endif
