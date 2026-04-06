#pragma once
#if defined(STM32G4) || defined(STM32_G)

#include "stm32g4xx_hal.h"

#include "voltbro/encoders/generic.h"
#include "voltbro/utils.hpp"

#include <memory>

#ifndef ARM_MATH_CM4
constexpr float PI = 3.14159265359;
#endif

struct CommonDriverConfig {
    const uint8_t ppairs = 1;
    const uint8_t gear_ratio = 1;
};

constexpr float FLOAT_INF = std::numeric_limits<float>::infinity();

struct DriveLimits {
    float current_limit = NAN;  // Real current limit for operation, can be modified by stall detection, etc.

    float user_current_limit = NAN;
    float user_torque_limit = NAN;
    float user_speed_limit = FLOAT_INF;
    float user_position_lower_limit = -FLOAT_INF;
    float user_position_upper_limit = FLOAT_INF;
    float user_angle_offset = 0.0f;
    int8_t user_angle_direction = 1;
};

class AbstractMotor {
protected:
    DriveLimits drive_limits;
    AbstractMotor() {}
public:
    virtual HAL_StatusTypeDef init() = 0;
    virtual HAL_StatusTypeDef stop() = 0;
    virtual HAL_StatusTypeDef start() = 0;
    virtual HAL_StatusTypeDef set_state(bool) = 0;
    virtual void update() = 0;

    virtual HAL_StatusTypeDef apply_limits() {
        if (std::isnan(drive_limits.current_limit)) {
            drive_limits.current_limit = drive_limits.user_current_limit;
        }
        return HAL_OK;
    };

    virtual bool check_limits(const DriveLimits& limits) {
        if (!std::isnan(limits.user_position_lower_limit) && !std::isnan(limits.user_position_upper_limit) &&
            limits.user_position_upper_limit < limits.user_position_lower_limit) {
            return false;
        }
        return true;
    }

    bool set_limits(const DriveLimits& limits) {
        if (!check_limits(limits)) {
            return false;
        }
        drive_limits = limits;
        auto result = apply_limits();
        if (result != HAL_OK) {
            return false;
        }
        return true;
    }

    const DriveLimits& get_limits() const {
        return drive_limits;
    }
};

inline float calculate_angle_simple(
    const CommonDriverConfig& drive,
    GenericEncoder& encoder
) {
    float current_circle_part = (float)encoder.get_value() / (float)encoder.CPR;

    if (encoder.is_electrical) {
        float circle_part_per_revolution = 1.0f / drive.ppairs;
        int32_t revolutions = encoder.get_revolutions() % (int16_t)drive.ppairs;
        if (revolutions < 0) {
            revolutions += drive.ppairs;
        }
        current_circle_part = (revolutions + current_circle_part) * circle_part_per_revolution;
    }

    return current_circle_part * pi2;
}

#endif
