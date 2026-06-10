#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED)

#include "voltbro/utils.hpp"

class BasicServo {
private:
    const tim_register min_phase;
    TIM_HandleTypeDef * const htim;
    const tim_register channel;
    bool _is_on = false;

public:
    BasicServo(TIM_HandleTypeDef *htim, tim_register channel, tim_register min_phase):
        min_phase(min_phase), htim(htim), channel(channel)
    {}
    BasicServo(TIM_HandleTypeDef *htim, tim_register channel):
            BasicServo(htim, channel, 1000)
    {}

    virtual void set_pulse(float pulse) {
        if (!_is_on) {
            // Защита от дурака
            __HAL_TIM_SET_COMPARE(htim, channel, min_phase);
            return;
        }
        assert_param((pulse >= 0.0) && (pulse <= 1.0));
        const tim_register adj = (float)min_phase * pulse;
        __HAL_TIM_SET_COMPARE(htim, channel, min_phase + adj);
    }

    void stop() {
        set_pulse(0);
        _is_on = false;
    }

    float get_pulse() const {
        const tim_register cpr = __HAL_TIM_GET_COMPARE(htim, channel);
        const tim_register offset_cpr = cpr - min_phase;
        return (float)offset_cpr / (float)min_phase;
    }

    bool is_on() const {
        return _is_on;
    }

    void start() {
        _is_on = true;
        set_pulse(0);
        HAL_TIM_PWM_Start(htim, channel);
    }
};

#endif
#endif
