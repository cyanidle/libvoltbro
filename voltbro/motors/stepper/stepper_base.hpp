#pragma once
#if defined(STM32G4) || defined(STM32_G)

#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED)

#include <utility>

#include "voltbro/devices/gpio.hpp"
#include "voltbro/utils.hpp"
#include "voltbro/motors/motor_commons.hpp"

class StepperBase : public AbstractMotor {
protected:
    arm_atomic(bool) _is_on = false;
    GpioPin en_pin;
public:
    StepperBase(GpioPin&& en_pin):
        AbstractMotor(),
        en_pin(std::move(en_pin))
    {}

    HAL_StatusTypeDef set_state(bool state) override{
        if (state) {
            en_pin.reset();
        }
        else {
            en_pin.set();
        }
        _is_on = state;
        return HAL_OK;
    }
    HAL_StatusTypeDef stop() override {
        return set_state(false);
    }
    HAL_StatusTypeDef start() override {
        return set_state(true);
    }
    bool is_on() const {
        return _is_on;
    }

    void update() override {}
};

#endif
#endif
