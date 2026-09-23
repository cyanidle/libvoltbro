#include "hall_sensor.h"
#if defined(STM32G4) || defined(STM32_G)

static FORCE_INLINE bool get_pin_state(const GPIO_TypeDef* gpiox, pin p) {
    return HAL_GPIO_ReadPin((GPIO_TypeDef *) gpiox, p) == GPIO_PIN_SET;
}

HallSensor::HallSensor(
    encoder_data CPR,
    bool is_inverted,
    GPIO_TypeDef *pin_1_gpiox,
    pin pin_1,
    GPIO_TypeDef *pin_2_gpiox,
    pin pin_2,
    GPIO_TypeDef *pin_3_gpiox,
    pin pin_3,
    HallSequence&& sequence,
    bool is_exti_trusted
):
    GenericEncoder(CPR, is_inverted, true),
    IS_EXTI_TRUSTED(is_exti_trusted),
    pin_1_gpiox(pin_1_gpiox),
    pin_2_gpiox(pin_2_gpiox),
    pin_3_gpiox(pin_3_gpiox),
    pin_1(pin_1),
    pin_2(pin_2),
    pin_3(pin_3),
    sequence(std::move(sequence))
    {
    // NOTE: deliberately no GPIO reads here. This object is usually
    // constructed statically, before GPIO clocks and pins are configured, so
    // sampling the pins here yields garbage and caches a bogus step. Call
    // resync() once after GPIO init instead.
    increment = is_inverted ? -1 : 1;
    build_transition_table();
}

void HallSensor::build_transition_table() {
    const uint8_t weight[3] = {
        to_underlying(sequence[0]),
        to_underlying(sequence[1]),
        to_underlying(sequence[2])
    };
    // Positive-direction channel activation cycle (the convention previously
    // encoded in handle_hall_channel): after channel 0 activates the next
    // positive transition flips channel 2, then 1, then 0 again.
    const uint8_t next_channel[3] = {2, 0, 1};

    // Find the unique valid 6-state cycle: try every (start state, last
    // activated channel) pair and keep the one where six consecutive
    // positive-direction flips visit only valid states and return to start.
    for (uint8_t start = 1; start <= 6; start++) {
        for (uint8_t start_channel = 0; start_channel < 3; start_channel++) {
            uint8_t state = start;
            uint8_t channel = start_channel;
            uint8_t visited[6];
            bool ok = true;
            for (int i = 0; i < 6; i++) {
                channel = next_channel[channel];
                state = (uint8_t)(state ^ weight[channel]);
                if (!is_valid_raw_state(state)) {
                    ok = false;
                    break;
                }
                visited[i] = state;
            }
            if (!ok || state != start) {
                continue;
            }
            // The found cycle is the positive direction; the same cycle
            // walked backwards is the negative one.
            uint8_t from = start;
            for (int i = 0; i < 6; i++) {
                const uint8_t to = visited[i];
                transition_table[from][to] = 1;
                transition_table[to][from] = -1;
                from = to;
            }
            return;
        }
    }
}

void HallSensor::resync() {
    CRITICAL_SECTION(
        state_1 = get_pin_state(pin_1_gpiox, pin_1);
        state_2 = get_pin_state(pin_2_gpiox, pin_2);
        state_3 = get_pin_state(pin_3_gpiox, pin_3);
        const uint8_t raw = get_raw_state();
        raw_state = raw;
        step = EncoderStep(raw);
        step_is_valid = is_valid_raw_state(raw);
    )
}

void HallSensor::update_value() {
    // Polling repairs a missed EXTI even while stopped. Serialize the complete
    // sample/count update with the ISR so a duplicate sample cannot count twice.
    CRITICAL_SECTION(
        handle_hall_channel();
    )
}

bool HallSensor::handle_hall_channel(pin channel) {
    if (IS_EXTI_TRUSTED && channel != NONE_UINT16) {
        // Sample the indicated channel instead of toggling its cache: polling
        // may already have observed this edge before the pending ISR runs.
        if (pin_1 == channel) {
            state_1 = get_pin_state(pin_1_gpiox, pin_1);
        }
        else if (pin_2 == channel) {
            state_2 = get_pin_state(pin_2_gpiox, pin_2);
        }
        else if (pin_3 == channel) {
            state_3 = get_pin_state(pin_3_gpiox, pin_3);
        }
        else {
            return false;  // not one of our channels
        }
    }
    else {
        state_1 = get_pin_state(pin_1_gpiox, pin_1);
        state_2 = get_pin_state(pin_2_gpiox, pin_2);
        state_3 = get_pin_state(pin_3_gpiox, pin_3);
    }

    const uint8_t new_raw = get_raw_state();
    const uint8_t old_raw = raw_state;
    raw_state = new_raw;
    step = EncoderStep(new_raw);
    step_is_valid = is_valid_raw_state(new_raw);

    if (new_raw == old_raw) {
        return false;
    }

    const int8_t new_direction = transition_table[old_raw][new_raw];
    if (new_direction == 0) {
        // Not a legal single six-step transition: a same-channel retrigger
        // (EXTI glitch) or several channels changed at once (missed edges).
        // The state was re-synchronised above; just don't count the event.
        return false;
    }
    direction = new_direction;

    int32_t signed_value = (int32_t)value + increment * direction;
    if (signed_value >= CPR) {
        signed_value = signed_value - CPR;
        incr_revolutions();
    } else if (signed_value < 0) {
        signed_value = signed_value + CPR;
        decr_revolutions();
    }
    uint16_t unsigned_value = static_cast<uint16_t>(signed_value);

    bool has_changed = value != unsigned_value;
    value = unsigned_value;

    return has_changed;
}

#endif
