#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#ifdef HAL_SPI_MODULE_ENABLED

#include <cstdint>

#include "voltbro/encoders/generic.h"
#include "voltbro/utils.hpp"
#include "voltbro/devices/gpio.hpp"
#include "voltbro/generics/spi_mixin.hpp"

template<class ASxxxxParams>
class ASxxxx: public GenericEncoder, public SPIMixin {
private:
    GpioPin spi_ss;

    FORCE_INLINE void start_transaction() {
        spi_ss.reset();
    }

    FORCE_INLINE void end_transaction() {
        spi_ss.set();
    }

    uint16_t get_command(ASxxxxParams reg) {
        uint16_t command = to_underlying(ASxxxxParams::COM_READ);
        command = command | (uint16_t)reg;

        uint8_t parity = 0;
        uint16_t n = command;
        while (n) {
            parity = !parity;
            n = n & (n - 1);
        }

        ASxxxxParams parity_bit;
        if (parity) {
            parity_bit = ASxxxxParams::PARITY_BIT_1;
        } else {
            parity_bit = ASxxxxParams::PARITY_BIT_0;
        }
        command = command | to_underlying(parity_bit);

        return command;
    }

    uint16_t read(bool recieve_immediate = false) {
        start_transaction();
        uint16_t response = spi_transmit_command_receive(to_underlying(ASxxxxParams::READ_MESSAGE));
        end_transaction();

        if (!recieve_immediate) {
            delay_cpu_cycles(to_underlying(ASxxxxParams::CS_DELAY_CYCLES));

            start_transaction();
            response = spi_transmit_command_receive((uint16_t)ASxxxxParams::REG_NOP);
            end_transaction();
        }

        last_error = (response & to_underlying(ASxxxxParams::ERROR_BIT));

        // Return the data, stripping the parity and error bits
        return response & ~to_underlying(ASxxxxParams::CLEAR_ERROR_AND_PARITY);
    }

    encoder_data get_angle(bool recieve_immediate = true) {
        uint16_t angle = read(recieve_immediate);
        if (is_inverted) {
            angle = CPR - angle;
        }
        return angle;
    }

public:
    ASxxxx(
        GpioPin&& spi_ss,
        SPI_HandleTypeDef* spi,
        encoder_data CPR,
        bool is_inverted = false,
        encoder_data electric_offset = 0
    ):
        GenericEncoder(CPR, is_inverted, false, electric_offset),
        SPIMixin(spi),
        spi_ss(std::move(spi_ss))
    {
        value = (encoder_data)-1;
    };

    void start_streaming() {
        // Warm up encoder
        for (size_t i = 0; i < 5; i++) {
            start_transaction();
            spi_transmit_only(get_command(ASxxxxParams::REG_ANGLE));
            end_transaction();
            delay_cpu_cycles(to_underlying(ASxxxxParams::CS_DELAY_CYCLES));
        }
    }

    void update_value() override {
        encoder_data new_value = get_angle();

        if (value == (encoder_data)-1) {
            value = new_value;
            return;
        }

        int32_t diff = (int32_t)value - (int32_t)new_value;
        const encoder_data half_cpr = CPR / 2;
        if (abs(diff) > half_cpr) {
            if (diff < 0) {
                decr_revolutions();
            }
            else {
                incr_revolutions();
            }
        }

        value = new_value;
    }

    HAL_StatusTypeDef init() override {
        HAL_StatusTypeDef result = GenericEncoder::init();
        if (result != HAL_OK) {
            return result;
        }
        LL_SPI_Enable(spi->Instance);
        start_streaming();
        return HAL_OK;
    }
};

#endif
#endif
