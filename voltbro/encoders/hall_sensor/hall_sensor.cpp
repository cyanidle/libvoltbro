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
    state_1 = get_pin_state(pin_1_gpiox, pin_1);
    state_2 = get_pin_state(pin_2_gpiox, pin_2);
    state_3 = get_pin_state(pin_3_gpiox, pin_3);
    increment = is_inverted ? -1 : 1;
    step = get_encoder_step();
}

#ifdef DEBUG
int8_t activated_pin;
#endif
bool HallSensor::handle_hall_channel(pin channel) {
#ifndef DEBUG
    uint8_t activated_pin;
#endif

    if (IS_EXTI_TRUSTED && channel != NONE_UINT16) {
        if (pin_1 == channel) {
            activated_pin = 0;
        } else if (pin_2 == channel) {
            activated_pin = 1;
        } else {
            activated_pin = 2;
        }
        *(&state_1 + activated_pin) = !*(&state_1 + activated_pin);
    }
    else {
        bool old_state_1 = state_1;
        bool old_state_2 = state_2;
        bool old_state_3 = state_3;
        state_1 = get_pin_state(pin_1_gpiox, pin_1);
        state_2 = get_pin_state(pin_2_gpiox, pin_2);
        state_3 = get_pin_state(pin_3_gpiox, pin_3);
        if (old_state_1 != state_1) {
            activated_pin = 0;
        }
        else if (old_state_2 != state_2) {
            activated_pin = 1;
        }
        else if (old_state_3 != state_3) {
            activated_pin = 2;
        }
        else {
            activated_pin = NONE_UINT8;
        }
    }

    if (activated_pin == NONE_UINT8) {
        return false;
    }

    if (last_activated == NONE_UINT8) {
        last_activated = activated_pin;
        step = get_encoder_step();
        return true;
    }

    int32_t signed_value = (int32_t)value;
    bool positive_direction = false;
    switch (last_activated) {
        case 0:
            positive_direction = activated_pin == 2;
            break;
        case 1:
            positive_direction = activated_pin == 0;
            break;
        case 2:
            positive_direction = activated_pin == 1;
            break;
    }
    if (positive_direction) {
        direction = 1;
    } else {
        direction = -1;
    }
    signed_value += increment * direction;
    last_activated = activated_pin;

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
    step = get_encoder_step();

    return has_changed;
}

#endif
