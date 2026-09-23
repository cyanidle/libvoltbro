#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#include <memory>

#include "../bldc.h"

/**
 * Simple controller with 6-step switching logic.
 */
class SixStepController: public BLDCController {
public:
    enum class Fault : uint8_t { NONE, INVALID_HALL, INVALID_SUPPLY, INVALID_COMMAND, STALLED };

private:
    HallSensor& hall_sensor;

    // --- velocity estimation state (hall transitions over time) ---
    int32_t last_counts = 0;          // last observed absolute encoder position, in counts
    uint32_t last_velocity_tick = 0;  // HAL_GetTick() at last velocity update, ms
    bool velocity_initialized = false;
    // No hall transition for this long => shaft is considered stopped.
    static constexpr uint32_t VELOCITY_TIMEOUT_MS = 100;

    // Three 40 ms kicks, separated by 200 ms Hall-controlled attempts, fit
    // inside a one-second drive pulse. A full electrical turn replenishes the
    // retry budget; Hall-edge chatter alone must not do so.
    static constexpr uint32_t NO_PROGRESS_MS = 200;
    static constexpr uint32_t KICK_MS = 40;
    static constexpr uint8_t MAX_KICKS = 3;
    Fault fault = Fault::NONE;
    int8_t recovery_direction = 0;
    int32_t progress_counts = 0;
    int32_t kick_origin_counts = 0;
    uint32_t progress_tick = 0;
    uint32_t kick_tick = 0;
    uint8_t recovery_attempts = 0;
    bool kicking = false;
    EncoderStep kick_step = EncoderStep::AB;

    void reset_recovery();
    void update_velocity(int32_t counts);
public:
    SixStepController(
        DriveRuntimeConfig& runtime_config,
        DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        BaseInverter& inverter,
        HallSensor& hall_sensor
    ):
        BLDCController(
            runtime_config,
            drive_info,
            htim,
            inverter
        ),
        hall_sensor(hall_sensor)
    {}

    FORCE_INLINE void flow_direction(DrivePhase from, DrivePhase to, int16_t pwm) {
        uint16_t actual_pwm = abs(pwm);
        if (pwm < 0) {
            DrivePhase tmp = to;
            to = from;
            from = tmp;
        }
        DrivePhase off;
        if (DrivePhase::PHASE_A != from && DrivePhase::PHASE_A != to)
            off = DrivePhase::PHASE_A;
        if (DrivePhase::PHASE_B != from && DrivePhase::PHASE_B != to)
            off = DrivePhase::PHASE_B;
        if (DrivePhase::PHASE_C != from && DrivePhase::PHASE_C != to)
            off = DrivePhase::PHASE_C;

        drive_info.l_pins.value()[to_underlying(off)].reset();
        DQs[to_underlying(off)] = 0;

        drive_info.l_pins.value()[to_underlying(from)].set();
        DQs[to_underlying(from)] = actual_pwm;

        drive_info.l_pins.value()[to_underlying(to)].set();
        DQs[to_underlying(to)] = 0;
    }

    void update() override;
    bool set_voltage_point(float voltage) override;

    Fault get_fault() const { return fault; }
    bool is_recovering() const { return kicking; }
    uint8_t get_recovery_attempts() const { return recovery_attempts; }

    // Freewheel the motor: all low-side inputs off, all duty cycles zeroed
    // and pushed to the timer compare registers.
    void coast();
};

// Maps a commutation step to the phase pair to energise. Returns false for
// an invalid step (raw hall 000/111); first/second are left untouched then.
bool step_to_phases(EncoderStep step, DrivePhase& first, DrivePhase& second);

#endif
#endif
