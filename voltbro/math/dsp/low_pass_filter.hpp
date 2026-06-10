#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx.h"
#endif

#ifdef ARM_MATH_CM4
#include <arm_math.h>
#else
#include <math.h>
#endif

#include "voltbro/utils.hpp"
#include "filter.hpp"

class LowPassFilter: public AbstractFilter {
public:
    const float beta;
    LowPassFilter(float beta = 0.5): beta(beta) {};

    inline float operator () (float prev_value, float new_value) const override {
        if (is_close(beta, 1)) {
            return new_value;
        }
        return (float)((1.0f - beta) * prev_value + beta * new_value);
    }
};
