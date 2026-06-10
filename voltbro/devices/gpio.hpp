#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"

#include <voltbro/utils.hpp>

class GpioPin {
public:
    GpioPin(GPIO_TypeDef* port, pin pin_) : port_(port), pin_(pin_) {}

    inline void set() const {
        HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
    }

    inline void reset() const {
        HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
    }

    inline void toggle() const {
        HAL_GPIO_TogglePin(port_, pin_);
    }

    inline GPIO_PinState read() const {
        return HAL_GPIO_ReadPin(port_, pin_);
    }

    inline bool isSet() const {
        return read() == GPIO_PIN_SET;
    }

    inline bool isReset() const {
        return !isSet();
    }

    GPIO_TypeDef* getPort() const { return port_; }
    uint16_t getPin() const { return pin_; }

private:
    GPIO_TypeDef* port_;
    uint16_t pin_;
};

#endif
