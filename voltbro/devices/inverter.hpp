#pragma once

#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_conf.h"
#ifdef HAL_ADC_MODULE_ENABLED
#include <cstdint>


#ifdef MONITOR
static volatile float I_A_glob, I_B_glob, I_C_glob, busV_glob;
#endif

class BaseInverter {
protected:
    float I_A_offset = 0;
    float I_B_offset = 0;
    float I_C_offset = 0;
    float I_A = 0;
    float I_B = 0;
    float I_C = 0;
    float busV = 0;

    bool is_started = false;
public:
    float get_A() const { return I_A; }
    float get_B() const { return I_B; }
    float get_C() const { return I_C; }
    float get_busV() const { return busV; }
    bool has_started() const { return is_started; }

    virtual void start() = 0;
    virtual void update() = 0;
    virtual ~BaseInverter() = default;
};


class Inverter: public BaseInverter {
private:
    const volatile uint32_t ADC_buffer[4] = {};
    ADC_HandleTypeDef* hadc;

public:
    Inverter(ADC_HandleTypeDef* hadc): hadc(hadc) {}

    void start() override {
        if (!has_started()) {
            return;
        }

        // apply factory calibration
        HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
        // const_cast to allow HAL to work
        HAL_ADC_Start_DMA(hadc, const_cast<uint32_t *>(ADC_buffer), 4);
        // Record offset;
        int cycles = 64;
        for (int i = 0; i < cycles; i++) {
            I_A_offset += (3.3f * (float)ADC_buffer[1] / (16.0f * 4096.0f));
            I_B_offset += (3.3f * (float)ADC_buffer[2] / (16.0f * 4096.0f));
            I_C_offset += (3.3f * (float)ADC_buffer[3] / (16.0f * 4096.0f));
            HAL_Delay(1);
        }
        I_A_offset /= (float)cycles;
        I_B_offset /= (float)cycles;
        I_C_offset /= (float)cycles;

        is_started = true;
    }

    void update() override {
        const float shunt_res = 0.045f;  // 0.045 Ohm shunt resistance
        const float op_amp_gain = 1.0f;  // current sensor gain
        I_A = ((3.3f * (float)ADC_buffer[1] / (16.0f * 4096.0f)) - I_A_offset) /
              (shunt_res * op_amp_gain);
        I_B = ((3.3f * (float)ADC_buffer[2] / (16.0f * 4096.0f)) - I_B_offset) /
              (shunt_res * op_amp_gain);
        I_C = ((3.3f * (float)ADC_buffer[3] / (16.0f * 4096.0f)) - I_C_offset) /
              (shunt_res * op_amp_gain);
        busV = 16.0f * 3.3f * (float)ADC_buffer[0] / (16.0f * 4096.0f);  // drivers input voltage
        #ifdef MONITOR
            I_A_glob = I_A;
            I_B_glob = I_B;
            I_C_glob = I_C;
            busV_glob = busV;
        #endif
    }
};

#endif
#endif
