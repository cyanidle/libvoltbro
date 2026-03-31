#include "voltbro/devices/stspin32g4.hpp"

#if defined(STM32G4) || defined(STM32_G)
#ifdef HAL_I2C_MODULE_ENABLED

HAL_StatusTypeDef STSPIN32G4::write_lock_register(uint8_t value) const {
    return HAL_I2C_Mem_Write(
        hi2c,
        device_address,
        to_address(Register::LOCK),
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        register_timeout_ms
    );
}

HAL_StatusTypeDef STSPIN32G4::is_device_ready(uint32_t trials, uint32_t timeout_ms) const {
    return HAL_I2C_IsDeviceReady(hi2c, device_address, trials, timeout_ms);
}

HAL_StatusTypeDef STSPIN32G4::write_register(Register reg, uint8_t value) const {
    return HAL_I2C_Mem_Write(
        hi2c,
        device_address,
        to_address(reg),
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        register_timeout_ms
    );
}

HAL_StatusTypeDef STSPIN32G4::clear_faults() const {
    return write_register(Register::CLEAR, command_execute_value);
}

HAL_StatusTypeDef STSPIN32G4::wake(uint32_t timeout_ms) const {
    wake_pin.set();

    const uint32_t start_tick = HAL_GetTick();
    HAL_StatusTypeDef status = HAL_ERROR;
    do {
        status = is_device_ready(1U, 1U);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        HAL_Delay(1U);
    } while ((HAL_GetTick() - start_tick) < timeout_ms);

    return status;
}

HAL_StatusTypeDef STSPIN32G4::request_standby() const {
    wake_pin.reset();

    HAL_StatusTypeDef status = write_lock_register(lock_unlock_value);
    if (status != HAL_OK) {
        return status;
    }

    const uint8_t standby_request = standby_bit;
    status = write_register(Register::STANDBY, standby_request);

    static_cast<void>(write_lock_register(lock_locked_value));
    return status;
}

#endif
#endif
