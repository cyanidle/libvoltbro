#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED)

#include "voltbro/utils.hpp"

class Impeller {
private:
    TIM_HandleTypeDef * const htim;
    const tim_register channel;
    bool _is_on = false;
public:
    Impeller(TIM_HandleTypeDef *htim, tim_register channel): htim(htim), channel(channel) {}

    // 0 <= p <= 1
    void set_power(float p) {
        if (!_is_on) {
            // Защита от дурака
            __HAL_TIM_SET_COMPARE(htim, channel, 0);
            return;
        }
        assert_param(p >= 0.0f and p <= 1.0f);
        const tim_register pulse = htim->Instance->ARR * p;
        __HAL_TIM_SET_COMPARE(htim, channel, pulse);
    }

    void stop() {
        set_power(0);
        _is_on = false;
    }

    bool is_on() const {
        return _is_on;
    }

    void start() {
        _is_on = true;
        set_power(0);
        HAL_TIM_PWM_Start(htim, channel);
    }
};

#endif
#endif
