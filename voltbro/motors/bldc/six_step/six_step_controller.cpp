#include "six_step_controller.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#define USE_CONTROL

void SixStepController::update_velocity() {
    // Absolute shaft position in encoder counts. revolutions tracks full turns and
    // get_value() the position within a turn, so this is continuous across wraps.
    // Read revolutions + value as a consistent pair: the hall EXTI ISR bumps
    // both together at the CPR wrap, so a torn read there jumps counts by ±CPR
    // and produces a one-sample velocity spike. Retry until revolutions is
    // stable across the value read.
    int32_t rev, val;
    do {
        rev = hall_sensor.get_revolutions();
        val = (int32_t)hall_sensor.get_value();
    } while (rev != hall_sensor.get_revolutions());
    const int32_t counts = rev * (int32_t)hall_sensor.CPR + val;
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

void SixStepController::update() {
    update_velocity();

    if (!_is_on) {
        return;
    }
    inverter.update();

    DrivePhase first, second;
    if (!hall_sensor.is_step_valid() ||
        !step_to_phases(hall_sensor.get_step(), first, second)) {
        // Invalid or never-sampled hall state (000/111): freewheel instead
        // of commutating on uninitialised phases.
        coast();
        return;
    }

    // TODO: check and report if point_type is not voltage?
    const float bus_v = inverter.get_busV();  // TODO: busV or manually set supply_voltage?
    const float target_voltage = target;
    float pwm_f = 0.0f;
    if (std::isfinite(bus_v) && (bus_v > 0.0f) && std::isfinite(target_voltage)) {
        pwm_f = ((float)full_pwm / bus_v) * target_voltage;
    }
    // else: no usable bus measurement or target - drive zero PWM safely.

    // Clamp in the float domain BEFORE narrowing to int16_t; clamping after
    // the conversion cannot catch out-of-range floats (UB on overflow).
    const float max_pwm = (float)full_pwm * 0.95f;
    pwm_f = std::clamp(pwm_f, -max_pwm, max_pwm);
    int16_t new_pwm = (int16_t)std::lrintf(pwm_f);

#ifndef USE_CONTROL
    local_pwm = 300;
#endif

    if (hall_sensor.is_inverted) {
        new_pwm = (int16_t)-new_pwm;
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
