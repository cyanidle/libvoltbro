#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED)

#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif

#include "voltbro/utils.hpp"
#include "voltbro/encoders/generic.h"

class DCIncrementalEncoder : public GenericEncoder {
private:
    const encoder_data offset_incr;
    const uint16_t half_cpr;
    TIM_HandleTypeDef* const encoder_tim;
    int32_t offset = 0;

    /* WARNING! Explicitly specify alignment for guaranteed atomic reads and writes. Explanation:
     * https://developer.arm.com/documentation/dui0375/g/C-and-C---Implementation-Details/Basic-data-types-in-ARM-C-and-C-- or https://stackoverflow.com/a/52785864
     * short version: all reads/writes to var are atomic if it is "self"-aligned (1/2/4 byte)
     * (Please, copy this comment to all variables that can be accessed concurrently - as a warning and a reminder) */
    arm_atomic(float) processed_value;
public:
    DCIncrementalEncoder(
        TIM_HandleTypeDef* encoder_tim,
        encoder_data CPR,
        bool is_inverted = false
    ):
        GenericEncoder(CPR, is_inverted),
        offset_incr((UINT16_MAX + 1) % CPR),
        half_cpr(CPR / 2),
        encoder_tim(encoder_tim)
    {};

    HAL_StatusTypeDef init() override {
        return HAL_TIM_Encoder_Start(encoder_tim,TIM_CHANNEL_ALL);
    }

    inline encoder_data get_value() const override {
        return processed_value;
    }

    void update_value() override {
        encoder_data new_value = __HAL_TIM_GetCounter(encoder_tim);
        bool is_counting_down = __HAL_TIM_IS_TIM_COUNTING_DOWN(encoder_tim);

        int32_t diff = (int32_t)value - (int32_t)new_value;
        if (abs(diff) > INT16_MAX) { // INT16_MAX == UINT16_MAX / 2
            offset += copysign(offset_incr, diff);
        }
        value = new_value;

        uint32_t offset_val = value + offset;
        int32_t new_processed_value = offset_val % CPR - electric_offset;
        if (new_processed_value < 0) {
            new_processed_value += CPR;
        }

        int32_t difference = processed_value - new_processed_value;
        if (abs(difference) > half_cpr) {
            if (is_counting_down) {
                decr_revolutions();
            }
            else {
                incr_revolutions();
            }
        }

        processed_value = new_processed_value;
    }
};

#endif
#endif
