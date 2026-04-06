#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#include <cstdint>
#include <optional>
#include <array>
#include <algorithm>
#include <cmath>

#include "voltbro/utils.hpp"
#include "voltbro/math/math_ops.hpp"
#include "voltbro/math/regulators/pid.hpp"
#include "voltbro/encoders/hall_sensor/hall_sensor.h"
#include "voltbro/devices/gpio.hpp"
#include "voltbro/devices/inverter.hpp"
#include "../motor_commons.hpp"

enum class SetPointType: uint8_t  {
    VELOCITY = 0,
    TORQUE = 1,
    POSITION = 2,
    VOLTAGE = 3,
    UNIVERSAL = 4
};

enum class DrivePhase: uint8_t { PHASE_A = 0, PHASE_B = 1, PHASE_C = 2 };

struct DriveInfo {
    const float torque_const;
    float max_current;
    float max_torque;
    float stall_current;
    float stall_timeout;
    float stall_tolerance;
    float calibration_voltage;

    const std::optional<std::array<GpioPin, 3>> l_pins = std::nullopt;
    const GpioPin en_pin;
    const CommonDriverConfig common;
};


/*
 * Base for BLDC motors. Controls PWM, stall detection, current sensing, limits
 */
class BLDCController: public AbstractMotor {
protected:
    FORCE_INLINE bool is_symmetric_limit_set(float limit) const {
        return std::isfinite(limit) && (limit > 0.0f);
    }

    FORCE_INLINE bool is_within_symmetric_limit(float value, float limit) const {
        return !is_symmetric_limit_set(limit) || (std::fabs(value) <= std::fabs(limit));
    }

    FORCE_INLINE float get_torque_limit_from_current() const {
        if (!is_symmetric_limit_set(drive_limits.user_current_limit)) {
            return NAN;
        }
        return drive_limits.user_current_limit * drive_info.torque_const *
               static_cast<float>(drive_info.common.gear_ratio);
    }

    FORCE_INLINE float get_effective_torque_limit() const {
        const float torque_from_current = get_torque_limit_from_current();
        const float torque_from_limit =
            is_symmetric_limit_set(drive_limits.user_torque_limit) ?
                drive_limits.user_torque_limit :
                NAN;

        if (std::isnan(torque_from_limit)) {
            return torque_from_current;
        }
        if (std::isnan(torque_from_current)) {
            return torque_from_limit;
        }
        return std::min(std::fabs(torque_from_limit), std::fabs(torque_from_current));
    }

    FORCE_INLINE bool is_angle_target_valid(float angle) const {
        const float lower = drive_limits.user_position_lower_limit;
        const float upper = drive_limits.user_position_upper_limit;
        if (std::isfinite(lower) && (angle < lower)) {
            return false;
        }
        if (std::isfinite(upper) && (angle > upper)) {
            return false;
        }
        return true;
    }

    FORCE_INLINE bool is_velocity_target_valid(float velocity) const {
        return is_within_symmetric_limit(velocity, drive_limits.user_speed_limit);
    }

    FORCE_INLINE bool is_torque_target_valid(float torque) const {
        return is_within_symmetric_limit(torque, get_effective_torque_limit());
    }

    FORCE_INLINE float get_direction_multiplier() const {
        return static_cast<float>(drive_limits.user_angle_direction);
    }

    const DriveInfo drive_info;
    BaseInverter& inverter;
    const int32_t full_pwm;
    TIM_HandleTypeDef* const htim;

    arm_atomic(SetPointType) point_type = SetPointType::VOLTAGE;
    arm_atomic(float) target = 0;

    arm_atomic(float) shaft_angle;
    arm_atomic(float) shaft_velocity;
    arm_atomic(float) shaft_torque = 0;
    arm_atomic(bool) _is_on;
    arm_atomic(bool) is_stalling;
    uint16_t DQs[3] = {0, 0, 0};
public:
    BLDCController(
        const DriveLimits& limits,
        const DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        BaseInverter& inverter
    ):
        AbstractMotor(),
        drive_info(drive_info),
        inverter(inverter),
        full_pwm(htim->Instance->ARR),
        htim(htim)
    {
        if (!set_limits(limits)) {
            exit(-1);
        }
    }

    virtual bool check_limits(const DriveLimits& limits) override {
        if ((limits.user_angle_direction != -1) && (limits.user_angle_direction != 1)) {
            return false;
        }
        if (
            std::isfinite(limits.user_speed_limit) &&
            (limits.user_speed_limit < 0.0f)
        ) {
            return false;
        }
        if (
            std::isfinite(limits.user_current_limit) &&
            (limits.user_current_limit > drive_info.max_current)
        ) {
            return false;
        }
        if (
            std::isfinite(limits.user_torque_limit) &&
            (limits.user_torque_limit > drive_info.max_torque)
        ) {
            return false;
        }
        return AbstractMotor::check_limits(limits);
    }
    virtual HAL_StatusTypeDef apply_limits() override {
        if (std::isnan(drive_limits.user_current_limit) || drive_limits.user_current_limit <= 0) {
            drive_limits.user_current_limit = drive_info.max_current;
        }
        if (std::isnan(drive_limits.user_torque_limit) || drive_limits.user_torque_limit <= 0) {
            drive_limits.user_torque_limit = drive_info.max_torque;
        }
        return AbstractMotor::apply_limits();
    }

    FORCE_INLINE virtual bool set_angle_point(float angle) {
        if (!is_angle_target_valid(angle)) {
            return false;
        }
        point_type = SetPointType::POSITION;
        target = angle;
        return true;
    }

    FORCE_INLINE virtual bool set_velocity_point(float velocity) {
        if (!is_velocity_target_valid(velocity)) {
            return false;
        }
        point_type = SetPointType::VELOCITY;
        target = velocity;
        return true;
    }
    FORCE_INLINE virtual bool set_torque_point(float torque) {
        if (!is_torque_target_valid(torque)) {
            return false;
        }
        point_type = SetPointType::TORQUE;
        target = torque * get_direction_multiplier();
        return true;
    }
    FORCE_INLINE virtual bool set_voltage_point(float voltage) {
        point_type = SetPointType::VOLTAGE;
        target = voltage * get_direction_multiplier();
        return true;
    }
    const DriveInfo& get_info() const {
        return drive_info;
    }
    const BaseInverter& get_inverter() const {
        return inverter;
    }
    FORCE_INLINE bool is_on() const {
        return _is_on;
    }
    FORCE_INLINE float get_angle() const {
        return shaft_angle * get_direction_multiplier() + drive_limits.user_angle_offset;
    }
    FORCE_INLINE float get_velocity() const {
        return shaft_velocity * get_direction_multiplier();
    }
    FORCE_INLINE float get_voltage() const {
        return inverter.get_busV();
    }
    FORCE_INLINE virtual float get_torque() const {
        return shaft_torque * get_direction_multiplier();
    }

    void detect_stall(double passed_time_abs);
    void quit_stall();
    HAL_StatusTypeDef init() override;
    HAL_StatusTypeDef stop() override;
    HAL_StatusTypeDef start() override;
    HAL_StatusTypeDef set_state(bool) override;

    FORCE_INLINE virtual void set_pwm() {
        htim->Instance->CCR1 = DQs[0];
        htim->Instance->CCR2 = DQs[1];
        htim->Instance->CCR3 = DQs[2];
    }
};

static FORCE_INLINE float get_current(Inverter& inverter, DrivePhase current_relative) {
    switch (current_relative) {
        case DrivePhase::PHASE_A:
            return -inverter.get_A();
        case DrivePhase::PHASE_B:
            return -inverter.get_B();
        case DrivePhase::PHASE_C:
            return -inverter.get_C();
    }
}

#endif
#endif
