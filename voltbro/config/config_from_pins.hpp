#pragma once
#if defined(STM32G4) || defined(STM32_G)

#include <memory>
#include <cstdint>

#include "stm32g4xx_hal.h"

#include "voltbro/utils.hpp"

typedef struct {
    pin pin_num;
    GPIO_TypeDef* port;
} PinInfo;

inline uint8_t pin_state(const PinInfo& pin_info) {
    GPIO_PinState state = HAL_GPIO_ReadPin(pin_info.port, pin_info.pin_num);
    return state == GPIO_PIN_SET ? 1 : 0;
}

template <size_t N>
class ConfigFromPins {
public:
    std::array<PinInfo, N> pins;
    ConfigFromPins(const std::array<PinInfo, N>&& pins): pins({pins}){}

    uint16_t get_id() {
        uint16_t result = 0;
        for (size_t i = 0; i < pins.size(); i++) {
            result += pin_state(pins[i]) << i;
        }
        return result;
    }
};

#endif
