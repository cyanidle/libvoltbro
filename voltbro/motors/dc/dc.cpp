#if defined(STM32G4) || defined(STM32_G)
#include "dc.h"
#if defined(HAL_DAC_MODULE_ENABLED) && defined(HAL_TIM_MODULE_ENABLED)

#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif

HAL_StatusTypeDef DCMotorController::set_state(bool state) {
    HAL_GPIO_WritePin(
        config.nSLEEP_GPIOx,
        config.nSLEEP_pin,
        state ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
    _is_on = state;
    return HAL_OK;
}

HAL_StatusTypeDef DCMotorController::apply_runtime_config() {
    Ipeak = drive_runtime_config.current_limit;
    double Vref = Ipeak * config.Rsense * 10 * 2;
    uint32_t dac_val = dac_value(Vref);
    auto state = HAL_DAC_SetValue(
        config.dac,
        config.dac_channel_,
        DAC_ALIGN_12B_R,
        dac_val
    );
    if (state != HAL_OK) {
        return state;
    }
    return AbstractMotor::apply_runtime_config();
};

void DCMotorController::set_pulse(float pwm) {
    auto compare_value = static_cast<uint32_t>(floorf(abs(pwm)));
    if (pwm < 0) {
        __HAL_TIM_SET_COMPARE(config.timer, config.IN2_channel, compare_value);
        __HAL_TIM_SET_COMPARE(config.timer, config.IN1_channel, 0);
    }
    else {
        __HAL_TIM_SET_COMPARE(config.timer, config.IN2_channel, 0);
        __HAL_TIM_SET_COMPARE(config.timer, config.IN1_channel, compare_value);
    }
}

HAL_StatusTypeDef DCMotorController::init() {
    encoder.init();

    HAL_DAC_Start(config.dac, config.dac_channel_);

    for (auto& channel: {config.IN1_channel, config.IN2_channel}) {
        auto status = HAL_TIM_PWM_Start(config.timer, channel);
        if (status != HAL_OK) {
            return status;
        }
    }
    __HAL_TIM_SET_COMPARE(config.timer, config.IN1_channel, 0);
    __HAL_TIM_SET_COMPARE(config.timer, config.IN2_channel, 0);

    return stop();
}

void DCMotorController::update() {
    encoder.update_value();
    angle = calculate_angle_simple(config.common, encoder);
}

#endif
#endif
