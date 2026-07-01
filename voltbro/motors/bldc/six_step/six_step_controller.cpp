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

void SixStepController::update() {
    update_velocity();

    if (!_is_on) {
        return;
    }
    inverter.update();

    DrivePhase first, second;
    step_to_phases(hall_sensor.get_step(), first, second);

    // TODO: check and report if point_type is not voltage?
    int16_t new_pwm = full_pwm / inverter.get_busV() * target;  // TODO: busV or manually set supply_voltage?

    const uint32_t MAX_PWM = full_pwm * 0.95f;
    if ( ((uint16_t)abs(new_pwm)) > MAX_PWM ) {
        new_pwm = copysign(MAX_PWM, new_pwm);
    }

#ifndef USE_CONTROL
    local_pwm = 300;
#endif

    if (hall_sensor.is_inverted) {
        new_pwm = -new_pwm;
    }
    flow_direction(first, second, new_pwm);

    set_pwm();
}

void step_to_phases(EncoderStep step, DrivePhase& first, DrivePhase& second) {
    switch (step) {
        case EncoderStep::AB:
            first = DrivePhase::PHASE_A;
            second = DrivePhase::PHASE_B;
            break;
        case EncoderStep::AC:
            first = DrivePhase::PHASE_A;
            second = DrivePhase::PHASE_C;
            break;
        case EncoderStep::BC:
            first = DrivePhase::PHASE_B;
            second = DrivePhase::PHASE_C;
            break;
        case EncoderStep::BA:
            first = DrivePhase::PHASE_B;
            second = DrivePhase::PHASE_A;
            break;
        case EncoderStep::CA:
            first = DrivePhase::PHASE_C;
            second = DrivePhase::PHASE_A;
            break;
        case EncoderStep::CB:
            first = DrivePhase::PHASE_C;
            second = DrivePhase::PHASE_B;
            break;
    }
}

#endif
