#pragma once
#if defined(STM32G474xx) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#include <utility>

#include "six_step_controller.h"
#include "voltbro/math/regulators/pid.hpp"

/*
 * Closed-loop angular-velocity controller for a six-step BLDC motor.
 *
 * Wraps a SixStepController and steers its open-loop voltage set-point with a
 * PID regulator. The feedback is the controller's own calculated shaft velocity
 * (SixStepController::get_velocity(), rad/s), so this turns the bare six-step
 * driver into a speed-regulated drive.
 *
 * Units:
 *   - target / feedback: shaft angular velocity, rad/s
 *   - PID output:        voltage set-point, V (fed to set_voltage_point)
 */
class SixStepSpeedController {
private:
    SixStepController& motor;
    PIDRegulator velocity_pid;
    // Desired shaft angular velocity, rad/s. Aligned for atomic access since it
    // can be written from a CAN callback and read from the control loop.
    arm_atomic(float) target_velocity = 0.0f;

public:
    SixStepSpeedController(SixStepController& motor, PIDConfig&& pid_config):
        motor(motor),
        velocity_pid(std::move(pid_config))
    {}

    void set_target_velocity(float velocity) { target_velocity = velocity; }
    float get_target_velocity() const { return target_velocity; }

    // Feedback value the regulator is acting on, rad/s.
    float get_velocity() const { return motor.get_velocity(); }

    // Retune the regulator at run time (e.g. from a config message).
    void set_pid_config(PIDConfig&& config) { velocity_pid.update_config(std::move(config)); }

    /*
     * Run one regulation step. dt is the time since the previous call, seconds.
     * Computes the velocity error and pushes the resulting voltage set-point to
     * the wrapped controller; the controller's own update() then drives the PWM.
     */
    void update(float dt) {
        const float error = target_velocity - motor.get_velocity();
        // zero_in_threshold = false: hold the computed voltage even when the
        // error is small, otherwise the wheel would coast inside the tolerance.
        const float voltage = velocity_pid.regulation(error, dt, false);
        motor.set_voltage_point(voltage);
    }
};

#endif
#endif
