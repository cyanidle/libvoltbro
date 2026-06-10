#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"

#include <cstdint>

#include "voltbro/utils.hpp"
#include "voltbro/math/dsp/low_pass_filter.hpp"

class GenericEncoder {
protected:
    bool last_error = false;

    /* WARNING! Explicitly specify alignment for guaranteed atomic reads and writes. Explanation:
     * https://developer.arm.com/documentation/dui0375/g/C-and-C---Implementation-Details/Basic-data-types-in-ARM-C-and-C-- or https://stackoverflow.com/a/52785864
     * short version: all reads/writes to var are atomic if it is "self"-aligned (1/2/4 byte)
     * (Please, copy this comment to all variables that can be accessed concurrently - as a warning and a reminder) */
    arm_atomic(int) revolutions = 0;
    arm_atomic(encoder_data) value = 0;
public:
    const int electric_offset = 0;
    const encoder_data CPR;
    const bool is_electrical = false;
    const bool is_inverted = false;

    GenericEncoder(
        encoder_data CPR,
        bool is_inverted = false,
        bool is_electrical = false,
        int electric_offset = 0
    ):
        electric_offset(electric_offset),
        CPR(CPR),
        is_electrical(is_electrical),
        is_inverted(is_inverted)
        {};

    virtual HAL_StatusTypeDef init() {
        return HAL_OK;
    }
    virtual void update_value() {}

    virtual FORCE_INLINE encoder_data get_value() const {
        return value;
    };

    FORCE_INLINE void incr_revolutions() {
        revolutions++;
    }
    FORCE_INLINE void decr_revolutions() {
        revolutions--;
    }

    FORCE_INLINE int get_revolutions() const{
        return revolutions;
    }

};

#endif
