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

struct DriveRuntimeConfig {
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
    DriveRuntimeConfig drive_runtime_config;
    AbstractMotor() {}
public:
    virtual HAL_StatusTypeDef init() = 0;
    virtual HAL_StatusTypeDef stop() = 0;
    virtual HAL_StatusTypeDef start() = 0;
    virtual HAL_StatusTypeDef set_state(bool) = 0;
    virtual void update() = 0;

    virtual HAL_StatusTypeDef apply_runtime_config() {
        if (std::isnan(drive_runtime_config.current_limit)) {
            drive_runtime_config.current_limit = drive_runtime_config.user_current_limit;
        }
        return HAL_OK;
    };

    virtual bool check_runtime_config(const DriveRuntimeConfig& runtime_config) {
        if (
            !std::isnan(runtime_config.user_position_lower_limit) &&
            !std::isnan(runtime_config.user_position_upper_limit) &&
            runtime_config.user_position_upper_limit < runtime_config.user_position_lower_limit
        ) {
            return false;
        }
        return true;
    }

    bool set_runtime_config(const DriveRuntimeConfig& runtime_config) {
        if (!check_runtime_config(runtime_config)) {
            return false;
        }
        drive_runtime_config = runtime_config;
        auto result = apply_runtime_config();
        if (result != HAL_OK) {
            return false;
        }
        return true;
    }

    const DriveRuntimeConfig& get_runtime_config() const {
        return drive_runtime_config;
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
