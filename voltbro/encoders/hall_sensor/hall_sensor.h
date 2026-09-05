#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"

#include <array>
#include <cstdint>
#include <limits>

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

constexpr uint16_t NONE_UINT16 = std::numeric_limits<uint16_t>::max();

class HallSensor: public GenericEncoder {
public:
    using HallSequence = std::array<HallPhase, 3>;
private:

    const bool IS_EXTI_TRUSTED;

    int state_1 = 0;
    int state_2 = 0;
    int state_3 = 0;
    int8_t increment = 1;
    // Written from the hall EXTI ISR, read from the control loop: volatile
    // for visibility across the ISR boundary (aligned word accesses are
    // atomic on Cortex-M4, but without volatile they may be cached).
    volatile EncoderStep step = EncoderStep::AB;
    // False until a real sample of the hall inputs produces a valid state
    // (000/111 are not reachable with 120-degree hall placement).
    volatile bool step_is_valid = false;
    int8_t direction = 0;
    // Raw 3-bit hall state of the last processed event (0 before the first
    // sample / resync()).
    volatile uint8_t raw_state = 0;
    const GPIO_TypeDef* pin_1_gpiox;
    const GPIO_TypeDef* pin_2_gpiox;
    const GPIO_TypeDef* pin_3_gpiox;
    const pin pin_1;
    const pin pin_2;
    const pin pin_3;
    HallSequence sequence;

    // transition_table[from][to]: +1 if from->to is a positive-direction
    // commutation transition, -1 if negative, 0 if it is not a legal single
    // six-step transition (same-channel retrigger, multi-channel jump).
    int8_t transition_table[8][8] = {};

    static constexpr bool is_valid_raw_state(uint8_t raw) {
        return raw >= 1 && raw <= 6;
    }

    FORCE_INLINE uint8_t get_raw_state() const {
        return (uint8_t)(
            state_1 * to_underlying(sequence[0]) +
            state_2 * to_underlying(sequence[1]) +
            state_3 * to_underlying(sequence[2])
        );
    }

    // Fills transition_table from `sequence` (pure math, no hardware access).
    void build_transition_table();
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

    /*
     * (Re)synchronise with the physical hall inputs. The constructor does NOT
     * read GPIO (the sensor is typically constructed statically, before the
     * GPIO clocks/pins are configured, so the read would be garbage); call
     * this once after GPIO init, before relying on get_step() or counting.
     */
    void resync();

    FORCE_INLINE EncoderStep get_step() const { return step; }
    // False while no valid hall state has been sampled (or the inputs read
    // 000/111): the controller must coast instead of commutating.
    FORCE_INLINE bool is_step_valid() const { return step_is_valid; }

    bool handle_hall_channel(pin channel = NONE_UINT16);

    void update_value() override {}
    inline encoder_data get_value() const override {
        return value;
    }
};

#endif
