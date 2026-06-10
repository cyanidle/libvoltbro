#pragma once
#if defined(STM32G4) || defined(STM32_G)

#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED)

#include <utility>

#include "stepper_base.hpp"

struct StepperSimpleConfig {
    const GpioPin direction;
    const GpioPin sd_mode;
    const GpioPin spi_mode;

    const GpioPin cfg0;
    const GpioPin cfg1;
    const GpioPin cfg2;
    const GpioPin cfg3;
    const GpioPin cfg4;
    const GpioPin cfg5;
    const GpioPin cfg6;

    const pwm_channel step_channel;
    TIM_HandleTypeDef* const timer;
};

class StepperMotorSimple : public StepperBase {
protected:
    const StepperSimpleConfig config;
    const uint32_t counts_per_sec;
    const uint32_t PULSE_WIDTH = 10;
    arm_atomic(bool) _is_on = false;
public:
    StepperMotorSimple(StepperSimpleConfig&& driver, GpioPin&& enn_pin):
        StepperBase(std::forward<GpioPin&&>(enn_pin)),
        config(std::move(driver)),
        counts_per_sec(160000000 / (config.timer->Instance->PSC + 1))
    {};

    HAL_StatusTypeDef start_pwm() {
        return HAL_TIM_PWM_Start(config.timer, config.step_channel);
    }

    HAL_StatusTypeDef init() override {
        config.sd_mode.set();
        config.spi_mode.reset();

        config.cfg0.reset();
        config.cfg1.reset();

        config.cfg2.reset();
        config.cfg3.reset();
        config.cfg4.reset();

        config.cfg5.set();
        config.cfg6.set();

        return start_pwm();
    }

    void set_pulse_freq(uint32_t freq, bool positive_direction) {
        if (is_close(freq, 0)) {
            __HAL_TIM_SET_COMPARE(config.timer, config.step_channel, 0);
            return;
        }
        else {
            __HAL_TIM_SET_COMPARE(config.timer, config.step_channel, PULSE_WIDTH);
        }

        auto arr_value = static_cast<uint32_t>(floorf(counts_per_sec / freq));
        if (arr_value <= PULSE_WIDTH) {

        }
        if (positive_direction) {
            config.direction.set();
        }
        else {
            config.direction.reset();
        }
        config.timer->Instance->ARR = arr_value;
    }
};

#endif
#endif
