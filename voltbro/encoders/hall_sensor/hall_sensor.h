#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"

#include <array>
#include <cstdint>

#include "voltbro/encoders/generic.h"
#include "voltbro/utils.hpp"

enum class HallPhase: uint8_t { PHASE_A = 1, PHASE_B = 2, PHASE_C = 4 };

enum class EncoderStep: uint8_t {
    CA = 4,
    AB = 1,
    CB = 5,
    BC = 2,
    BA = 6,
    AC = 3
};

constexpr uint8_t NONE_UINT8 = std::numeric_limits<uint8_t>::max();
constexpr uint16_t NONE_UINT16 = std::numeric_limits<uint16_t>::max();

class HallSensor: public GenericEncoder {
public:
    using HallSequence = std::array<HallPhase, 3>;
private:

    const bool IS_EXTI_TRUSTED;

    int state_1;
    int state_2;
    int state_3;
    int8_t increment;
    EncoderStep step;
    int8_t direction = 0;
    uint8_t last_activated = NONE_UINT8;
    const GPIO_TypeDef* pin_1_gpiox;
    const GPIO_TypeDef* pin_2_gpiox;
    const GPIO_TypeDef* pin_3_gpiox;
    const pin pin_1;
    const pin pin_2;
    const pin pin_3;
    HallSequence sequence;

    FORCE_INLINE EncoderStep get_encoder_step() {
        return EncoderStep(
            state_1 * to_underlying(sequence[0]) +
            state_2 * to_underlying(sequence[1]) +
            state_3 * to_underlying(sequence[2])
        );
    }
public:
    HallSensor(
        encoder_data CPR,
        bool is_inverted,
        GPIO_TypeDef* pin_1_gpiox,
        pin pin_1,
        GPIO_TypeDef* pin_2_gpiox,
        pin pin_2,
        GPIO_TypeDef* pin_3_gpiox,
        pin pin_3,
        HallSequence&& sequence,
        bool is_exti_trusted=false
    );

    EncoderStep get_step() const { return step; }
    bool handle_hall_channel(pin channel = NONE_UINT16);

    void update_value() override {}
    inline encoder_data get_value() const override {
        return value;
    }
};

#endif
