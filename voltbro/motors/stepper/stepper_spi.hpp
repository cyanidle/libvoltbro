#pragma once
#if defined(STM32G4) || defined(STM32_G)

#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_SPI_MODULE_ENABLED)

#include <array>
#include <bit>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "stepper_base.hpp"
#include "voltbro/generics/spi_mixin.hpp"

struct StepperSPIConfig {
    const GpioPin direction;
    const GpioPin sd_mode;
    const GpioPin spi_mode;

    const GpioPin cfg4;
    const GpioPin cfg5;
    const GpioPin cfg6;

    GpioPin spi_ss;
    SPI_HandleTypeDef* spi;

    const pwm_channel step_channel;
    TIM_HandleTypeDef* const timer;
};

namespace tmc5160 {

namespace detail {
    constexpr void runtime_check(bool condition) {
        if (!std::is_constant_evaluated()) {
            assert_param(condition);
        }
    }
    [[nodiscard]] constexpr uint32_t pack_ihold_irun(
        uint8_t ihold,
        uint8_t irun,
        uint8_t iholddelay
    ) {
        return (static_cast<uint32_t>(iholddelay & 0x0F) << 16)
            | (static_cast<uint32_t>(irun & 0x1F) << 8)
            | static_cast<uint32_t>(ihold & 0x1F);
    }

    [[nodiscard]] constexpr uint32_t pack_chopconf_mres(uint8_t mres_code) {
        return static_cast<uint32_t>(mres_code) << 24;
    }
}

enum class MicrostepResolution: uint8_t {
    microsteps_256 = 0,  // Native internal-motion-controller resolution
    microsteps_128 = 1,
    microsteps_64 = 2,
    microsteps_32 = 3,
    microsteps_16 = 4,
    microsteps_8 = 5,
    microsteps_4 = 6,
    microsteps_2 = 7,
    full_step = 8
};

struct RegisterWrite {
    uint8_t address;
    uint32_t value;
};

struct RegisterConfig {
    // IHOLD is the standstill current scale.
    // Range: 0..31, where 0=1/32 and 31=32/32 of the configured full-scale current.
    uint8_t ihold = 3;

    // IRUN is the motion current scale.
    // Range: 0..31, using the same current-scale units as IHOLD.
    uint8_t irun = 6;

    // IHOLDDELAY controls how gradually the current decays from IRUN to IHOLD after TPOWERDOWN expires.
    // Range: 0..15.
    uint8_t iholddelay = 7;

    // Microstep resolution programmed into CHOPCONF.MRES.
    // With the internal ramp generator, native 256 microsteps is the least surprising setting.
    MicrostepResolution microstep_resolution = MicrostepResolution::microsteps_256;

    // A1 is the first acceleration phase between VSTART and V1.
    // Range: 0..65535 [usteps / ta^2]. A1=0 disables the dedicated A1/D1 phase.
    uint16_t a1 = 1000;

    // V1 is the threshold velocity where the ramp switches between A1/D1 and AMAX/DMAX.
    // Range: 0..(2^20)-1 [usteps / t].
    uint32_t v1 = 100000;

    // AMAX is the acceleration above V1.
    // Range: 0..65535 [usteps / ta^2].
    uint16_t amax = 50000;

    // VMAX is the target cruise velocity of the ramp generator.
    // Range: 0..(2^23)-512 [usteps / t].
    uint32_t vmax = 100000;

    // DMAX is the deceleration above V1.
    // Range: 0..65535 [usteps / ta^2]. If A1=0 disables the dedicated A1/D1 phase, DMAX becomes the active deceleration setting.
    uint16_t dmax = 50000;

    // D1 is the first deceleration phase between V1 and VSTOP.
    // Range: 1..65535 [usteps / ta^2] in positioning mode. Do not use 0 there.
    uint16_t d1 = 1400;

    // VSTOP is the near-zero stop velocity used at the end of a positioning move.
    // Range: 1..(2^18)-1 [usteps / t] in positioning mode. Datasheet recommends non-zero; 10 is a common safe default.
    uint32_t vstop = 10;
};

// Build the exact SPI register writes used to configure the TMC5160 ramp generator.
// Fixed settings intentionally kept out of RegisterConfig:
// - GCONF = 0: no global special modes enabled
// - CHOPCONF uses a conservative SpreadCycle baseline (TOFF=5, HSTRT=5, HEND=3, TBL=2)
// - TPOWERDOWN = 10
// - SW_MODE = 0
// - RAMPMODE = 0 (target position mode)
// - XACTUAL starts from 0
[[nodiscard]] constexpr auto build_register_config(
    const RegisterConfig& config
) -> std::array<RegisterWrite, 14> {
    detail::runtime_check(config.ihold <= 31);
    detail::runtime_check(config.irun <= 31);
    detail::runtime_check(config.iholddelay <= 15);
    detail::runtime_check(config.v1 <= 0x000FFFFF);
    detail::runtime_check(config.vmax <= 0x007FFE00);
    detail::runtime_check(config.d1 > 0);
    detail::runtime_check((config.vstop > 0) && (config.vstop <= 0x0003FFFF));

    constexpr uint32_t default_chopconf =
        detail::pack_chopconf_mres(static_cast<uint8_t>(MicrostepResolution::microsteps_256))
        | (static_cast<uint32_t>(2) << 15)
        | (static_cast<uint32_t>(3) << 7)
        | (static_cast<uint32_t>(5) << 4)
        | 5U;

    uint32_t chopconf = default_chopconf;
    chopconf &= ~(static_cast<uint32_t>(0x0F) << 24);
    chopconf |= detail::pack_chopconf_mres(static_cast<uint8_t>(config.microstep_resolution));

    return {{
        {0x80, 0x00000000},  // GCONF = 0: no special global flags enabled
        {0xEC, chopconf},  // CHOPCONF: SpreadCycle baseline with configured MRES, TOFF=5, HSTRT=5, HEND=3, TBL=2
        {0x90, detail::pack_ihold_irun(config.ihold, config.irun, config.iholddelay)},  // IHOLD_IRUN
        {0x91, 0x0000000A},  // TPOWERDOWN = 10: delay before dropping from run current after standstill is detected
        {0xF0, 0x00000000},  // SW_MODE = 0: no reference-switch actions, no soft stop, no StallGuard stop/latching enabled
        {0xA4, static_cast<uint32_t>(config.a1)},  // A1: first acceleration below V1
        {0xA5, config.v1},  // V1: threshold velocity between A1 and AMAX / D1 and DMAX
        {0xA6, static_cast<uint32_t>(config.amax)},  // AMAX: acceleration above V1
        {0xA7, config.vmax},  // VMAX: target cruise velocity for ramp motion
        {0xA8, static_cast<uint32_t>(config.dmax)},  // DMAX: deceleration above V1
        {0xAA, static_cast<uint32_t>(config.d1)},  // D1: first deceleration below V1
        {0xAB, config.vstop},  // VSTOP: near-zero stop velocity for positioning mode
        {0xA0, 0x00000000},  // RAMPMODE = 0: target-position mode using XTARGET
        {0xA1, 0x00000000}  // XACTUAL = 0: initialize current ramp-generator position to zero
    }};
}

}

class StepperMotorSPI : public StepperBase, protected SPIMixin {
protected:
    StepperSPIConfig config;
    const tmc5160::RegisterConfig register_config;
    arm_atomic(int32_t) position;

    template <size_t N>
    HAL_StatusTypeDef send_register_writes(const std::array<tmc5160::RegisterWrite, N>& register_writes) {
        for (const auto& register_write : register_writes) {
            if (write_register(register_write.address, register_write.value) != HAL_OK) {
                return HAL_ERROR;
            }
        }
        return HAL_OK;
    }

    HAL_StatusTypeDef transfer_datagram(uint8_t address, uint32_t value, uint8_t* spi_status, uint32_t* out) {
        const uint8_t tx_buffer[5] = {
            address,
            static_cast<uint8_t>((value >> 24) & 0xFF),
            static_cast<uint8_t>((value >> 16) & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>(value & 0xFF)
        };
        uint8_t rx_buffer[5] = {};

        config.spi_ss.reset();
        delay_cpu_cycles(CYCLES_100NS_160Mhz * 10 * 11);
        const HAL_StatusTypeDef status = spi_transmit_receive_buffer8(tx_buffer, rx_buffer, sizeof(tx_buffer));
        config.spi_ss.set();
        if (status != HAL_OK) {
            return status;
        }

        if (spi_status != nullptr) {
            *spi_status = rx_buffer[0];
        }
        if (out != nullptr) {
            *out = (static_cast<uint32_t>(rx_buffer[1]) << 24)
                | (static_cast<uint32_t>(rx_buffer[2]) << 16)
                | (static_cast<uint32_t>(rx_buffer[3]) << 8)
                | static_cast<uint32_t>(rx_buffer[4]);
        }
        return HAL_OK;
    }

    HAL_StatusTypeDef write_register(uint8_t address, uint32_t value) {
        return transfer_datagram(address, value, nullptr, nullptr);
    }

    HAL_StatusTypeDef read_register(uint8_t address, uint32_t* out) {
        uint8_t spi_status = 0;
        uint32_t discard = 0;
        HAL_StatusTypeDef status = transfer_datagram(address, 0x00000000, &spi_status, &discard);
        if (status != HAL_OK) {
            return status;
        }
        return transfer_datagram(address, 0x00000000, &spi_status, out);
    }
public:
    StepperMotorSPI(
        const StepperSPIConfig&& driver,
        GpioPin&& enn_pin,
        const tmc5160::RegisterConfig& register_config = {}
    ):
        StepperBase(std::forward<GpioPin&&>(enn_pin)),
        SPIMixin(driver.spi),
        config(std::move(driver)),
        register_config(register_config)
    {};

    int32_t get_position() {
        return position;
    }

    void update_position() {
        uint32_t raw_position = 0;
        if (read_register(0x21, &raw_position) == HAL_OK) {
            position = std::bit_cast<int32_t>(raw_position);
        }
    }

    virtual HAL_StatusTypeDef send_config() {
        const auto register_writes = tmc5160::build_register_config(register_config);
        return send_register_writes(register_writes);
    }

    HAL_StatusTypeDef init() override {
        config.sd_mode.reset();
        config.spi_mode.set();
        if (!LL_SPI_IsEnabled(config.spi->Instance)) {
            LL_SPI_Enable(config.spi->Instance);
        }
        HAL_Delay(150);
        return send_config();
    }

    virtual void set_target(int32_t value) {
        write_register(0xAD, std::bit_cast<uint32_t>(value));
    }

    void update() override {
        update_position();
    }
};

#endif
#endif
