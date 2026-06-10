#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#ifdef HAL_SPI_MODULE_ENABLED

#include "ASxxxx.hpp"

enum class AS5048AParams: uint16_t {
    // Registers
    REG_DIAG_AGC = 0x3FFD,
    REG_MAGNITUDE = 0x3FFE,
    REG_ANGLE = 0x3FFF,
    REG_CLEAR_ERROR_FLAG = 0x0001,
    REG_PROGRAMMING_CONTROL = 0x0003,
    REG_NOP = 0x0000,
    // Command basic values
    COM_READ = 0x4000,
    COM_WRITE = 0x0000,
    // Bits
    PARITY_BIT_1 = 0x8000,
    PARITY_BIT_0 = 0x0000,
    ERROR_BIT = 0x4000,
    CLEAR_ERROR_AND_PARITY = 0xC000,
    // Delays
    CS_DELAY_CYCLES = static_cast<uint16_t>(CYCLES_100NS_160Mhz * 3.5f),
    // Prepared messages
    READ_MESSAGE = 65535
};

class AS5048A final: public ASxxxx<AS5048AParams> {
public:
    AS5048A(
        GpioPin&& spi_ss,
        SPI_HandleTypeDef* spi,
        bool is_inverted = false,
        encoder_data electric_offset = 0
    ): ASxxxx<AS5048AParams>(
        std::forward<GpioPin>(spi_ss), spi, 16383, is_inverted, electric_offset
    ) {};
};

#endif
#endif
