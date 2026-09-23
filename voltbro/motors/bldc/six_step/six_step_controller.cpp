#include "six_step_controller.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

void SixStepController::update_velocity(int32_t counts) {
    const uint32_t now = HAL_GetTick();  // ms, SysTick time base

    // counts -> mechanical radians at the output shaft (after the gearbox)
    const float counts_to_rad = (pi2 / (float)hall_sensor.CPR) / (float)drive_info.common.gear_ratio;

    // Track mechanical shaft angle while we are here.
    shaft_angle = (float)counts * counts_to_rad;

    if (!velocity_initialized) {
        last_counts = counts;
        last_velocity_tick = now;
        velocity_initialized = true;
        return;
    }

    const int32_t dcount = counts - last_counts;
    const uint32_t dt_ms = now - last_velocity_tick;

    if (dcount != 0) {
        // Need at least 1ms of elapsed time for a meaningful rate; otherwise keep
        // accumulating counts (don't advance last_*) until the tick moves on.
        if (dt_ms == 0) {
            return;
        }
        const float dt = (float)dt_ms * 1e-3f;            // s
        const float measured = ((float)dcount * counts_to_rad) / dt;  // rad/s

        // EWMA low-pass: hall is coarse (CPR counts/rev), so smooth the quantisation.
        constexpr float alpha = 0.3f;
        shaft_velocity = shaft_velocity + alpha * (measured - shaft_velocity);

        last_counts = counts;
        last_velocity_tick = now;
    }
    else if (dt_ms >= VELOCITY_TIMEOUT_MS) {
        // No transitions for a while: the shaft has (nearly) stopped.
        shaft_velocity = 0.0f;
        last_velocity_tick = now;
    }
}

void SixStepController::coast() {
    if (drive_info.l_pins.has_value()) {
        for (const auto& pin : drive_info.l_pins.value()) {
            pin.reset();
        }
    }
    DQs[0] = DQs[1] = DQs[2] = 0;
    set_pwm();
}

void SixStepController::reset_recovery() {
    fault = Fault::NONE;
    recovery_direction = 0;
    recovery_attempts = 0;
    kicking = false;
}

bool SixStepController::set_voltage_point(float voltage) {
    if (voltage == 0.0f) {
        // Honor zero even if the next CAN command is drained in the same loop.
        // BLDCController::stop/start also go through this neutral set-point.
        reset_recovery();
        coast();
    }
    return BLDCController::set_voltage_point(voltage);
}

// The electrical cycle implied by the B-C-A Hall weighting of the application's
// wiring (see App/app.cpp); step_to_phases() below encodes the same assumption.
// A different Hall harness permutation would need this cycle rederived.
static EncoderStep next_step(EncoderStep step, bool reverse) {
    constexpr EncoderStep sequence[] = {
        EncoderStep::AB, EncoderStep::AC, EncoderStep::BC,
        EncoderStep::BA, EncoderStep::CA, EncoderStep::CB
    };
    for (unsigned i = 0; i < 6; ++i) {
        if (sequence[i] == step) {
            return sequence[(i + (reverse ? 5 : 1)) % 6];
        }
    }
    return step;  // invalid Hall states are rejected before recovery
}

void SixStepController::update() {
    hall_sensor.update_value();
    EncoderStep hall_step;
    int32_t counts;
    // The ISR updates both the turn count and within-turn count. Take one
    // coherent snapshot for commutation, velocity, and the progress watchdog.
    CRITICAL_SECTION(
        hall_step = hall_sensor.get_step();
        counts = hall_sensor.get_revolutions() * (int32_t)hall_sensor.CPR +
                 (int32_t)hall_sensor.get_value();
    )
    update_velocity(counts);

    const float target_voltage = target;
    if (!_is_on || target_voltage == 0.0f) {
        reset_recovery();
        coast();
        return;
    }

    // Repeated nonzero CAN commands (including sign changes) do not rearm a
    // failed startup. A zero command or explicit stop is required.
    if (fault == Fault::STALLED) {
        coast();
        return;
    }
    if (!std::isfinite(target_voltage)) {
        fault = Fault::INVALID_COMMAND;
        kicking = false;
        coast();
        return;
    }

    DrivePhase first, second;
    if (!step_to_phases(hall_step, first, second)) {
        fault = Fault::INVALID_HALL;
        kicking = false;
        coast();
        return;
    }

    inverter.update();
    const float bus_v = inverter.get_busV();
    if (!std::isfinite(bus_v) || bus_v <= 0.0f) {
        fault = Fault::INVALID_SUPPLY;
        kicking = false;
        coast();
        return;
    }
    fault = Fault::NONE;
    float pwm_f = ((float)full_pwm / bus_v) * target_voltage;

    // Clamp in the float domain BEFORE narrowing to int16_t; clamping after
    // the conversion cannot catch out-of-range floats (UB on overflow).
    const float max_pwm = (float)full_pwm * 0.95f;
    pwm_f = std::clamp(pwm_f, -max_pwm, max_pwm);
    int16_t new_pwm = (int16_t)std::lrintf(pwm_f);

    if (new_pwm == 0) {
        kicking = false;
        coast();
        return;
    }

    const uint32_t now = HAL_GetTick();
    const int8_t direction = target_voltage < 0.0f ? -1 : 1;
    if (direction != recovery_direction) {
        recovery_direction = direction;
        progress_counts = kick_origin_counts = counts;
        progress_tick = now;
        kicking = false;
    }
    if (direction * ((int64_t)counts - progress_counts) > 0) {
        progress_counts = counts;
        progress_tick = now;
        kicking = false;
        if (direction * ((int64_t)counts - kick_origin_counts) >= 6) {
            recovery_attempts = 0;
        }
    }

    if (kicking && (uint32_t)(now - kick_tick) >= KICK_MS) {
        kicking = false;
        progress_tick = now;
    }

    if (hall_sensor.is_inverted) {
        new_pwm = (int16_t)-new_pwm;
    }
    if (!kicking && (uint32_t)(now - progress_tick) >= NO_PROGRESS_MS) {
        if (recovery_attempts >= MAX_KICKS) {
            fault = Fault::STALLED;
            coast();
            return;
        }
        ++recovery_attempts;
        kicking = true;
        kick_tick = now;
        kick_origin_counts = counts;
        kick_step = next_step(hall_step, new_pwm < 0);
    }
    if (kicking) {
        // Shift the stator field by one electrical sector to escape an
        // equilibrium. Keep the requested voltage; never increase it blindly.
        step_to_phases(kick_step, first, second);
    }
    flow_direction(first, second, new_pwm);

    set_pwm();
}

bool step_to_phases(EncoderStep step, DrivePhase& first, DrivePhase& second) {
    switch (step) {
        case EncoderStep::AB:
            first = DrivePhase::PHASE_A;
            second = DrivePhase::PHASE_B;
            return true;
        case EncoderStep::AC:
            first = DrivePhase::PHASE_A;
            second = DrivePhase::PHASE_C;
            return true;
        case EncoderStep::BC:
            first = DrivePhase::PHASE_B;
            second = DrivePhase::PHASE_C;
            return true;
        case EncoderStep::BA:
            first = DrivePhase::PHASE_B;
            second = DrivePhase::PHASE_A;
            return true;
        case EncoderStep::CA:
            first = DrivePhase::PHASE_C;
            second = DrivePhase::PHASE_A;
            return true;
        case EncoderStep::CB:
            first = DrivePhase::PHASE_C;
            second = DrivePhase::PHASE_B;
            return true;
        default:
            return false;
    }
}

#endif
