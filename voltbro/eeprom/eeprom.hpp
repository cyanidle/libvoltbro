#pragma once
#if defined(STM32G4) || defined(STM32_G)

#include "stm32g4xx_hal.h"
#ifdef HAL_I2C_MODULE_ENABLED

#include <functional>

#include "voltbro/utils.hpp"

class EEPROM {
private:
    const uint16_t device_id = 0x50;
    const uint16_t page_size;
    const uint16_t mem_address_size;
    I2C_HandleTypeDef* i2c;

    using eeprom_operation = std::function<HAL_StatusTypeDef(
        I2C_HandleTypeDef*, uint16_t, uint16_t,
        uint16_t, uint8_t*, uint16_t, uint32_t
    )>;

    HAL_StatusTypeDef memory_op(eeprom_operation operation, uint8_t* bytes, size_t size, uint16_t address) {
        wait_until_available();

        uint16_t bytes_processed = 0;
        while (bytes_processed < size) {
            uint16_t page_offset = (address + bytes_processed) % page_size;
            uint16_t bytes_left_in_page = page_size - page_offset;
            uint16_t chunk_size = std::min(static_cast<uint16_t>(size - bytes_processed), bytes_left_in_page);

            HAL_StatusTypeDef result = operation(
                i2c,
                device_id << 1,
                address + bytes_processed,
                mem_address_size,
                bytes + bytes_processed,
                chunk_size,
                100
            );
            if (result != HAL_OK) {
                return result;
            }
            bytes_processed += chunk_size;
            wait_until_available();
        }

        return HAL_OK;
    }

public:
    explicit EEPROM(
        I2C_HandleTypeDef* i2c,
        uint64_t page_size = 64,
        uint16_t mem_address_size = I2C_MEMADD_SIZE_16BIT
    ): page_size(page_size), mem_address_size(mem_address_size), i2c(i2c) {};

    bool is_connected(void) {
        return HAL_I2C_IsDeviceReady(
            i2c,
            device_id << 1,
            2,
            100
        ) == HAL_OK;
    }

    void wait_until_available() {
        while (!is_connected()) {
            HAL_Delay(2);
        }
    }

    template <typename T>
    HAL_StatusTypeDef write(T* obj, uint16_t address) {
        auto bytes = reinterpret_cast<uint8_t*>(obj);
        size_t size_to_write = sizeof(T);
        return memory_op(HAL_I2C_Mem_Write, bytes, size_to_write, address);
    }

    template <typename T>
    HAL_StatusTypeDef read(T* obj, uint16_t address) {
        auto bytes = reinterpret_cast<uint8_t *>(obj);
        size_t size_to_read = sizeof(T);
        return memory_op(HAL_I2C_Mem_Read, bytes, size_to_read, address);
    }
};


#endif
#endif
