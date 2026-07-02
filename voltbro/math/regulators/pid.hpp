#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include "voltbro/utils.hpp"

struct PIDConfig {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral_error_lim = std::numeric_limits<float>::max();
    float max_output = std::numeric_limits<float>::max();
    float min_output = -std::numeric_limits<float>::max();
};

class PIDRegulator {
private:
    float signal = 0.0f;
    float integral_error = 0.0f;
    float prev_error = 0.0f;
    PIDConfig config;

    float regulation_impl(float error, float dt, float lower, float upper) {
        float error_diff = error - prev_error;
        prev_error = error;

        if (std::isnan(integral_error)) {
            integral_error = 0;
        }
        float new_integral = integral_error + error * dt;
        if (std::fabs(new_integral) > config.integral_error_lim) {
            new_integral = std::copysign(config.integral_error_lim, new_integral);
        }

        float kd_part = 0.0f;
        if (!is_close(config.kd, 0.0f) && !is_close(dt, 0.0f, 1e-8f)) {
            kd_part = config.kd * error_diff / dt;
        }
        float raw = config.kp * error + config.ki * new_integral + kd_part;

        // anti-windup: only commit integral if not saturated
        if (raw < upper && raw > lower) {
            integral_error = new_integral;
        }
        signal = std::clamp(raw, lower, upper);
        return signal;
    }

public:
    PIDRegulator() = default;
    explicit PIDRegulator(PIDConfig&& config) : config(std::move(config)) {}

    float regulation(float error, float dt) {
        return regulation_impl(error, dt, config.min_output, config.max_output);
    }

    float regulation(float error, float dt, float limit) {
        return regulation_impl(error, dt, -limit, limit);
    }

    void update_config(PIDConfig&& new_config) {
        config = std::move(new_config);
    }

    void update_config(float kp, float ki, float kd) {
        config.kp = kp;
        config.ki = ki;
        config.kd = kd;
    }
};
