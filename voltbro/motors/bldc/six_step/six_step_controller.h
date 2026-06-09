#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED)

#include <memory>

#include "../bldc.h"

/**
 * Simple controller with 6-step switching logic.
 */
class SixStepController: public BLDCController {
private:
    HallSensor& hall_sensor;
public:
    SixStepController(
        DriveRuntimeConfig& runtime_config,
        DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        BaseInverter& inverter,
        HallSensor& hall_sensor
    ):
        BLDCController(
            runtime_config,
            drive_info,
            htim,
            inverter
        ),
        hall_sensor(hall_sensor)
    {}

    FORCE_INLINE void flow_direction(DrivePhase from, DrivePhase to, int16_t pwm) {
        uint16_t actual_pwm = abs(pwm);
        if (pwm < 0) {
            DrivePhase tmp = to;
            to = from;
            from = tmp;
        }
        DrivePhase off;
        if (DrivePhase::PHASE_A != from && DrivePhase::PHASE_A != to)
            off = DrivePhase::PHASE_A;
        if (DrivePhase::PHASE_B != from && DrivePhase::PHASE_B != to)
            off = DrivePhase::PHASE_B;
        if (DrivePhase::PHASE_C != from && DrivePhase::PHASE_C != to)
            off = DrivePhase::PHASE_C;

        drive_info.l_pins.value()[to_underlying(off)].reset();
        DQs[to_underlying(off)] = 0;

        drive_info.l_pins.value()[to_underlying(from)].set();
        DQs[to_underlying(from)] = actual_pwm;

        drive_info.l_pins.value()[to_underlying(to)].set();
        DQs[to_underlying(to)] = 0;
    }

    void update() override;
};

void step_to_phases(EncoderStep step, DrivePhase& first, DrivePhase& second);

#endif
#endif
