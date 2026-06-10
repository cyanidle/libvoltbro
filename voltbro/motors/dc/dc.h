#pragma once
#if defined(STM32G4) || defined(STM32_G)

#include "stm32g4xx_hal.h"
#if defined(HAL_DAC_MODULE_ENABLED) && defined(HAL_TIM_MODULE_ENABLED)

#include <utility>
#include <memory>

#include "voltbro/utils.hpp"
#include "voltbro/encoders/generic.h"
#include "voltbro/motors/motor_commons.hpp"

struct DCDriverConfig {
    const pin nSLEEP_pin;
    GPIO_TypeDef* const nSLEEP_GPIOx;

    const pwm_channel IN1_channel;
    const pwm_channel IN2_channel;
    TIM_HandleTypeDef* const timer;

    const dac_channel dac_channel_;
    DAC_HandleTypeDef* const dac;

    const double Rsense;

    const CommonDriverConfig common;
};


class DCMotorController: public AbstractMotor {
private:
    const DCDriverConfig config;
    GenericEncoder& encoder;
    bool is_using_brake;

    /* WARNING! Explicitly specify alignment for guaranteed atomic reads and writes. Explanation:
     * https://developer.arm.com/documentation/dui0375/g/C-and-C---Implementation-Details/Basic-data-types-in-ARM-C-and-C-- or https://stackoverflow.com/a/52785864
     * short version: all reads/writes to var are atomic if it is "self"-aligned (1/2/4 byte)
     * (Please, copy this comment to all variables that can be accessed concurrently - as a warning and a reminder) */
    arm_atomic(bool) _is_on = false;
    arm_atomic(float) Ipeak = 0;
    arm_atomic(float) angle = 0;

    float current_pwm;
public:
    DCMotorController(
        const DCDriverConfig& driver_config,
        const DriveRuntimeConfig& runtime_config,
        GenericEncoder& encoder,
        bool is_using_brake = false
    ):
        AbstractMotor(),
        config(driver_config),
        encoder(encoder),
        is_using_brake(is_using_brake)
    {
        set_runtime_config(runtime_config);
    };

    HAL_StatusTypeDef init() override;
    HAL_StatusTypeDef set_state(bool) override;
    virtual HAL_StatusTypeDef apply_runtime_config() override;
    void update() override;
    void set_pulse(float pwm);

    bool is_on() const {
        return _is_on;
    }
    float get_angle() const {
        return angle;
    }
    HAL_StatusTypeDef stop() override {
        return set_state(false);
    }
    HAL_StatusTypeDef start() override {
        return set_state(true);
    }
};

#endif
#endif
