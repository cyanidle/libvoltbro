#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32g4xx_ll_spi.h"

#include <cstddef>

#include "voltbro/utils.hpp"

class SPIMixin {
protected:
    SPI_HandleTypeDef* const spi;

    explicit SPIMixin(SPI_HandleTypeDef* spi): spi(spi) {}

    bool spi_is_enabled() const {
        return spi != nullptr && spi->Instance != nullptr && LL_SPI_IsEnabled(spi->Instance);
    }

    void spi_clear_rx_fifo() {
        auto* spix = spi->Instance;
        while (LL_SPI_GetRxFIFOLevel(spix) != LL_SPI_RX_FIFO_EMPTY) {
            if (spi->Init.DataSize > SPI_DATASIZE_8BIT) {
                (void)LL_SPI_ReceiveData16(spix);
            }
            else {
                (void)LL_SPI_ReceiveData8(spix);
            }
        }
        if (LL_SPI_IsActiveFlag_OVR(spix)) {
            LL_SPI_ClearFlag_OVR(spix);
        }
    }

    void spi_prepare_transfer() {
        assert_param(spi_is_enabled());
        auto* spix = spi->Instance;
        while (LL_SPI_IsActiveFlag_BSY(spix)) {}
        spi_clear_rx_fifo();
    }

    void spi_finish_transfer() {
        auto* spix = spi->Instance;
        while (LL_SPI_IsActiveFlag_BSY(spix)) {}
        spi_clear_rx_fifo();
    }

    void spi_transmit_only(uint16_t command) {
        spi_prepare_transfer();
        auto* spix = spi->Instance;

        while (!LL_SPI_IsActiveFlag_TXE(spix)) {}
        LL_SPI_TransmitData16(spix, command);

        while (!LL_SPI_IsActiveFlag_RXNE(spix)) {}
        (void)LL_SPI_ReceiveData16(spix);

        spi_finish_transfer();
    }

    uint16_t spi_transmit_command_receive(uint16_t command) {
        spi_prepare_transfer();
        auto* spix = spi->Instance;

        while (!LL_SPI_IsActiveFlag_TXE(spix)) {}
        LL_SPI_TransmitData16(spix, command);

        while (!LL_SPI_IsActiveFlag_RXNE(spix)) {}
        uint16_t resp = LL_SPI_ReceiveData16(spix);

        spi_finish_transfer();
        return resp;
    }

    HAL_StatusTypeDef spi_transmit_receive_buffer8(const uint8_t* tx, uint8_t* rx, size_t size) {
        if (!spi_is_enabled()) {
            return HAL_ERROR;
        }

        spi_prepare_transfer();
        auto* spix = spi->Instance;

        for (size_t i = 0; i < size; i++) {
            while (!LL_SPI_IsActiveFlag_TXE(spix)) {}
            LL_SPI_TransmitData8(spix, tx[i]);

            while (!LL_SPI_IsActiveFlag_RXNE(spix)) {}
            rx[i] = LL_SPI_ReceiveData8(spix);
        }

        spi_finish_transfer();
        return HAL_OK;
    }
};

#endif
#endif
