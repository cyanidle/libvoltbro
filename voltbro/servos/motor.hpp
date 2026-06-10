#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED)

#include <utility>
#include <algorithm>

#include "basic.hpp"

constexpr float HALF_PI = PI / 2.0f;

class ServoMotor: public BasicServo {
public:
    typedef std::pair<float, float> Interval;
private:
    Interval safe_zone = {0, 1};
public:
    ServoMotor(TIM_HandleTypeDef *htim, tim_register channel, const Interval& _safe_zone):
        BasicServo(htim, channel, 1000),
        safe_zone(_safe_zone)
    {
        assert_param(safe_zone.first <= safe_zone.second);
    }
    ServoMotor(TIM_HandleTypeDef *htim, tim_register channel): ServoMotor(htim, channel, {0, 0}) {}

    float get_target_angle() {
        const float pulse = get_pulse();
        return pulse * PI - HALF_PI;
    }

    // -HALF_PI <= angle <= +HALF_PI
    void set_target_angle(float angle) {
        assert_param(angle >= -HALF_PI && angle <= +HALF_PI);
        set_pulse((angle + HALF_PI) / PI);
    }

    void set_pulse(float pulse) override {
        pulse = std::clamp(pulse, safe_zone.first, safe_zone.second);
        BasicServo::set_pulse(pulse);
    }
};

#endif
#endif
