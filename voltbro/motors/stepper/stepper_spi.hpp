#pragma once
#if defined(STM32G4) || defined(STM32_G)

#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_SPI_MODULE_ENABLED)

#include "stm32g4xx_ll_spi.h"

#include <array>
#include <bit>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "stepper_base.hpp"

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
) -> std::array<RegisterWrite, 15> {
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
        {0xA1, 0x00000000},  // XACTUAL = 0: initialize current ramp-generator position to zero
        {0x21, 0x00000000}  // Queue XACTUAL read so the next position poll returns a valid value
    }};
}

}

#define check_status(command) status = command; if (status != HAL_OK) { return status; }

class StepperMotorSPI : public StepperBase {
protected:
    StepperSPIConfig config;
    const tmc5160::RegisterConfig register_config;
    arm_atomic(bool) _is_on = false;
    arm_atomic(int32_t) position;

    template <size_t N>
    HAL_StatusTypeDef send_register_writes(const std::array<tmc5160::RegisterWrite, N>& register_writes) {
        uint32_t recv;
        HAL_StatusTypeDef status;
        for (const auto& register_write : register_writes) {
            check_status(send_recieve_data(register_write.address, register_write.value, &recv))
        }
        return HAL_OK;
    }
public:
    StepperMotorSPI(
        const StepperSPIConfig&& driver,
        GpioPin&& enn_pin,
        const tmc5160::RegisterConfig& register_config = {}
    ):
        StepperBase(std::forward<GpioPin&&>(enn_pin)),
        config(std::move(driver)),
        register_config(register_config)
    {};

    int32_t get_position() {
        return position;
    }

    HAL_StatusTypeDef send_recieve_data(uint8_t address, uint32_t in, uint32_t* out) {
        uint8_t tx_buffer[4];
        uint8_t rx_buffer[4];
        tx_buffer[0] = (in >> 24) & 0xFF;
        tx_buffer[1] = (in >> 16) & 0xFF;
        tx_buffer[2] = (in >> 8) & 0xFF;
        tx_buffer[3] = in & 0xFF;

        HAL_StatusTypeDef status;

        config.spi_ss.reset();
        delay_cpu_cycles(CYCLES_100NS_160Mhz * 10 * 11);
        while (!LL_SPI_IsActiveFlag_TXE(config.spi->Instance)) {}
        check_status(HAL_SPI_Transmit(config.spi, &address, 1, HAL_MAX_DELAY))

        while (!LL_SPI_IsActiveFlag_TXE(config.spi->Instance)) {}
        check_status(HAL_SPI_TransmitReceive(config.spi, tx_buffer, rx_buffer, 4, HAL_MAX_DELAY))
        config.spi_ss.set();

        uint32_t received_data = 0;
        received_data |= (uint32_t)rx_buffer[0] << 24;
        received_data |= (uint32_t)rx_buffer[1] << 16;
        received_data |= (uint32_t)rx_buffer[2] << 8;
        received_data |= (uint32_t)rx_buffer[3];
        *out = received_data;

        while (LL_SPI_IsActiveFlag_BSY(config.spi->Instance)) {}
        return HAL_OK;
    }

    void update_position() {
        uint32_t discard = 0;
        uint32_t raw_position = 0;

        // TMC5160 SPI reads are pipelined: the first transaction queues the XACTUAL read,
        // the second transaction returns its value. A previous write (for example XTARGET)
        // invalidates any earlier queued read, so this must be done on every poll.
        send_recieve_data(0x21, 0x00000000, &discard);
        send_recieve_data(0x21, 0x00000000, &raw_position);
        position = std::bit_cast<int32_t>(raw_position);
    }

    virtual HAL_StatusTypeDef send_config() {
        const auto register_writes = tmc5160::build_register_config(register_config);
        return send_register_writes(register_writes);
    }

    HAL_StatusTypeDef init() override {
        config.sd_mode.reset();
        config.spi_mode.set();
        HAL_Delay(150);
        return send_config();
    }

    virtual void set_target(int32_t value) {
        uint32_t recv;
        send_recieve_data(0xAD, std::bit_cast<uint32_t>(value), &recv);
    }

    void update() override {
        update_position();
    }
};

#endif
#endif
