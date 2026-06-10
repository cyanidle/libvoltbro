#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED)

#include "array"
#include "memory"
#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif

#include "voltbro/servos/basic.hpp"

static constexpr uint32_t arming_delay = 20;
static constexpr uint32_t arming_segments = 1 * 1000 / arming_delay;
static constexpr double eighth_pi = M_PI / 8;

class BLHeli_SController: public BasicServo {
public:
    static constexpr millis RECOMMENDED_DELAY = 1500;

    BLHeli_SController(TIM_HandleTypeDef *htim, tim_register channel):
        BasicServo(htim, channel, 1000)
    {}
};

inline void rising_arm(std::array<std::shared_ptr<BLHeli_SController>, 2>& controllers_iterable) {
    for (float t = 0; t < eighth_pi; t += eighth_pi / arming_segments) {
        for (auto& controller: controllers_iterable) {
            HAL_Delay(arming_delay);
            controller->set_pulse(sinf(t));
        }
    }
}

inline void falling_arm(std::array<std::shared_ptr<BLHeli_SController>, 2>& controllers_iterable) {
    for (float t = 0; t < eighth_pi; t += eighth_pi / arming_segments) {
        for (auto& controller: controllers_iterable) {
            HAL_Delay(arming_delay);
            controller->set_pulse(sinf(eighth_pi - t));
        }
    }
}

inline void rising_arm(std::shared_ptr<BLHeli_SController>& controller) {
    for (float t = 0; t < eighth_pi; t += eighth_pi / arming_segments) {
        HAL_Delay(arming_delay);
        controller->set_pulse(sinf(t));
    }
}

inline void falling_arm(std::shared_ptr<BLHeli_SController>& controller) {
    for (float t = 0; t < eighth_pi; t += eighth_pi / arming_segments) {
        HAL_Delay(arming_delay);
        controller->set_pulse(sinf(eighth_pi - t));
    }
}

inline void calibration_sequence(std::array<std::shared_ptr<BLHeli_SController>, 2>& controllers_iterable) {
    rising_arm(controllers_iterable);

    for (auto& controller: controllers_iterable) {
        controller->set_pulse(1.0f);
    }
    HAL_Delay(6000);
    for (auto& controller: controllers_iterable) {
        controller->set_pulse(0.5f);
    }
    HAL_Delay(500);
    for (auto& controller: controllers_iterable) {
        controller->set_pulse(0.0f);
    }
    HAL_Delay(4000);
}

inline void calibration_sequence(std::shared_ptr<BLHeli_SController>& controller) {
    //rising_arm(controller);

    controller->set_pulse(1.0f);
    HAL_Delay(6000);
    controller->set_pulse(0.5f);
    HAL_Delay(500);
    controller->set_pulse(0.0f);
    HAL_Delay(4000);
}

inline void arming_sequence(std::array<std::shared_ptr<BLHeli_SController>, 2>& controllers_iterable) {
    rising_arm(controllers_iterable);
    falling_arm(controllers_iterable);
    for (auto& controller: controllers_iterable) {
        controller->set_pulse(0.0f);
    }
    HAL_Delay(500);
}

inline void arming_sequence(std::shared_ptr<BLHeli_SController>& controller) {
    rising_arm(controller);
    falling_arm(controller);
    controller->set_pulse(0.0f);
    HAL_Delay(500);
}

#endif
#endif
