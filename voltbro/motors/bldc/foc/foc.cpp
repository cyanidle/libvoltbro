#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_TIM_MODULE_ENABLED) && defined(HAL_ADC_MODULE_ENABLED) && defined(HAL_CORDIC_MODULE_ENABLED)

#include "foc.hpp"

#include "arm_math.h"
#include "stm32g4xx_ll_cordic.h"

#include "voltbro/math/transform.hpp"

#ifdef FOC_PROFILE
struct FOCProfile {
    volatile uint32_t sensors = 0;
    volatile uint32_t currents = 0;
    volatile uint32_t outer_loop = 0;
    volatile uint32_t pwm = 0;
    volatile uint32_t total = 0;
};
static volatile FOCProfile foc_profile;

struct SensorsProfile {
    volatile uint32_t inverter = 0;
    volatile uint32_t angle = 0;
    volatile uint32_t kalman = 0;
    volatile uint32_t total = 0;
};
static volatile SensorsProfile sensors_profile;

struct KalmanProfile {
    volatile uint32_t start = 0;
    volatile uint32_t mid = 0;
    volatile uint32_t end = 0;
    volatile uint32_t total = 0;
};
static volatile KalmanProfile kalman_profile;
#endif

#if defined(MONITOR)
volatile float I_D = 0;
volatile float I_Q = 0;
static float V_d, V_q;
volatile float i_q_error, i_d_error;
float d_response, q_response, i_q_set;
volatile float control_error_glob = 0;
volatile float controller_response_glob = 0;
volatile float value_foc_p = 0;
volatile float value_foc_v = 0;
volatile float value_foc_p_kp = 0;
volatile float value_foc_v_kp = 0;
volatile float value_foc_t = 0;
volatile encoder_data raw_value = 0;
volatile float value_control_velocity = 0;
#endif

void FOC::update_angle() {
    encoder.update_value();

    #ifndef MONITOR
    encoder_data raw_value;
    #endif
    raw_value = encoder.get_value();
    int offset_value = (int)raw_value - encoder.electric_offset;
    if (lookup_table != nullptr) {
        offset_value -= (*lookup_table)[raw_value >> 3];
    }
    static const float cpr_offset = (float)encoder.CPR / (2.0f * drive_info.common.ppairs);
    offset_value -= cpr_offset;
    if(offset_value > (encoder.CPR - 1)) {
        offset_value -= encoder.CPR;
    }
    else if( offset_value < 0 ) {
        offset_value += encoder.CPR;
    }

    raw_elec_angle = offset_value * (pi2 / (float)encoder.CPR);
}

void FOC::apply_kalman() {
#ifdef FOC_PROFILE
    const uint32_t start_total = DWT->CYCCNT;
    uint32_t t_start = DWT->CYCCNT;
#endif
    static float prev_angle = -pi2 - 2;

    if (prev_angle < (-pi2 - 1)) {
        prev_angle = raw_elec_angle;
        return;
    }

    float travel = raw_elec_angle - prev_angle;
    if (travel < -PI) {
        travel += pi2;
    } else if (travel > PI) {
        travel -= pi2;
    }
#ifdef FOC_PROFILE
    kalman_profile.start = DWT->CYCCNT - t_start;
    t_start = DWT->CYCCNT;
#endif

//#pragma region KALMAN_PAPER
    /*
     * Source: "A digital speed filter for motion control drives
     *          with a low resolution position encoder",
     * AUTOMATIKA, 44(2003),
     * A. Bellini, S. Bifaretti, S. Constantini
     */
    // TODO: get acceleration from inverter?
    static float Th_hat = 0.0f; // Theta hat, rad
    static float W_hat = 0.0f; // Omega hat, rad/s
    static float E_hat = 0.0f; // Epsilon hat, rad/s^2

    // (11)
    float nTh = Th_hat + W_hat * T + (E_hat + filters_config.expected_a) * (T*T) / 2.0f;
    float nW = W_hat + (E_hat + filters_config.expected_a) * T;
    float nE = E_hat;

    nTh = mfmod(nTh, pi2);
    if( nTh < 0.0f ){
        nTh += pi2;
    }

    // (19)
    Th_hat = nTh + filters_config.g1 * travel;
    W_hat = nW + filters_config.g2 * travel;
    E_hat = nE + filters_config.g3 * travel;
//#pragma endregion KALMAN_PAPER
#ifdef FOC_PROFILE
    kalman_profile.mid = DWT->CYCCNT - t_start;
    t_start = DWT->CYCCNT;
#endif

    const float ab = pi2 / (float)drive_info.common.ppairs;
    elec_angle = (float)drive_info.common.ppairs * mfmod(nTh, ab);
    shaft_velocity = nW / drive_info.common.gear_ratio;

    prev_angle = nTh;
#ifdef FOC_PROFILE
    kalman_profile.end = DWT->CYCCNT - t_start;
    kalman_profile.total = DWT->CYCCNT - start_total;
#endif
}

void FOC::update_shaft_angle() {
    static float prev_elec_angle = elec_angle;
    static int32_t elec_turns = 0;
    float filtered_travel = elec_angle - prev_elec_angle;
    prev_elec_angle = elec_angle;
    if (filtered_travel < -PI) {
        elec_turns += 1;
    } else if (filtered_travel > PI) {
        elec_turns -= 1;
    }
    float elec_unwrapped = (float)elec_turns * pi2 + elec_angle;
    shaft_angle = elec_unwrapped / ( (float)drive_info.common.ppairs * drive_info.common.gear_ratio );
}

void FOC::update_sensors() {
#ifdef FOC_PROFILE
    const uint32_t start_total = DWT->CYCCNT;
    uint32_t t_start = DWT->CYCCNT;
#endif
    inverter.update();
#ifdef FOC_PROFILE
    sensors_profile.inverter = DWT->CYCCNT - t_start;
    t_start = DWT->CYCCNT;
#endif
    update_angle();
#ifdef FOC_PROFILE
    sensors_profile.angle = DWT->CYCCNT - t_start;
    t_start = DWT->CYCCNT;
#endif
    apply_kalman();
    update_shaft_angle();
    if (!is_control_velocity_initialized) {
        control_velocity = shaft_velocity;
        is_control_velocity_initialized = true;
    } else {
        control_velocity = velocity_lpf(control_velocity, shaft_velocity);
    }
    #ifdef MONITOR
    value_control_velocity = control_velocity;
    #endif
#ifdef FOC_PROFILE
    sensors_profile.kalman = DWT->CYCCNT - t_start;
    sensors_profile.total = DWT->CYCCNT - start_total;
#endif
}


void FOC::update() {
#ifdef FOC_PROFILE
    const uint32_t start_total = DWT->CYCCNT;
    uint32_t t_start = DWT->CYCCNT;
#endif
    update_sensors();
#ifdef FOC_PROFILE
    foc_profile.sensors = DWT->CYCCNT - t_start;
#endif

    // calculate sin and cos of electrical angle with the help of CORDIC.
    // convert electrical angle from float to q31. electrical theta should be [-pi, pi]
    int32_t ElecTheta_q31 = (int32_t)((elec_angle / PI - 1.0f) * 2147483648.0f);
    // load angle value into CORDIC. Input value is in PIs!
    LL_CORDIC_WriteData(CORDIC, ElecTheta_q31);

    // the values are negative to level out [-pi, pi] representation of electrical angle at the CORDIC input
    struct {
        int32_t cosOutput = -(int32_t)LL_CORDIC_ReadData(CORDIC);  // Read cosine
        int32_t sinOutput = -(int32_t)LL_CORDIC_ReadData(CORDIC);  // Read sine
    } elec_angles_q31;
    struct {
        float c;
        float s;
    } elec_angles;
    //arm_q31_to_float(&elec_angles_q31.cosOutput, &elec_angles.c, 2);

    elec_angles.c = (float32_t)elec_angles_q31.cosOutput / 2147483648.0f;  // convert to float from q31
    elec_angles.s = (float32_t)elec_angles_q31.sinOutput / 2147483648.0f;  // convert to float from q31

    #ifndef MONITOR
    float V_d, V_q;
    static float I_D = 0;
    #endif
    #ifdef FOC_PROFILE
    t_start = DWT->CYCCNT;
    #endif
    // LPF for motor current
    float tempD, tempQ;
    // dq0 transform on currents
    dq0(elec_angles.s, elec_angles.c, inverter.get_A(), inverter.get_B(), inverter.get_C(), &tempD, &tempQ);
    const float diff_D = I_D - tempD;
    const float diff_Q = I_Q - tempQ;

    I_D = I_D - (filters_config.I_lpf_coefficient * diff_D);
    I_Q = I_Q - (filters_config.I_lpf_coefficient * diff_Q);
    #ifdef FOC_PROFILE
    foc_profile.currents = DWT->CYCCNT - t_start;
    #endif

    const float gear_ratio_f = static_cast<float>(drive_info.common.gear_ratio);
    const float busV = inverter.get_busV();

    shaft_torque = I_Q * drive_info.torque_const * gear_ratio_f;

    if (point_type == SetPointType::VOLTAGE) {
        i_q_set_slewed = 0.0f;
        V_d = 0;
        V_q = target;
    }
    else {
        #ifndef MONITOR
        float i_d_error, i_q_error, d_response, q_response, i_q_set;
        #endif

        i_d_error = -I_D;
        d_response = d_reg.regulation(i_d_error, T, busV);
        V_d = std::clamp(d_response, -busV, busV);

        i_q_set = 0.0f;
        #ifdef FOC_PROFILE
        t_start = DWT->CYCCNT;
        #endif
        if (point_type == SetPointType::UNIVERSAL) {
            #ifdef MONITOR
            value_foc_p = foc_target.angle;
            value_foc_v = foc_target.velocity;
            value_foc_p_kp = foc_target.angle_kp;
            value_foc_v_kp = foc_target.velocity_kp;
            value_foc_t = foc_target.torque;
            #endif
            i_q_set = 1.0f / drive_info.torque_const * (
                foc_target.angle_kp * (foc_target.angle - get_angle()) +
                foc_target.velocity_kp * (foc_target.velocity - control_velocity) +
                (foc_target.torque / gear_ratio_f)
            );
        }
        else if (point_type == SetPointType::TORQUE) {
            i_q_set = target / drive_info.torque_const / gear_ratio_f;
        }
        else {
            float control_error = 0;
            if (point_type == SetPointType::POSITION) {
                control_error = target - get_angle();
            }
            else if (point_type == SetPointType::VELOCITY) {
                control_error = target - control_velocity;
            }
            float controller_response = control_reg.regulation(control_error, T, false);
            #ifdef MONITOR
            control_error_glob = control_error;
            controller_response_glob = controller_response;
            #endif
            i_q_set = controller_response * get_direction_multiplier();
        }

        const float abs_max_current_from_torque = (drive_info.max_torque / drive_info.torque_const / gear_ratio_f);
        if (fabs(i_q_set) > fabs(abs_max_current_from_torque)) {
            i_q_set = copysign(abs_max_current_from_torque, i_q_set);
        }
        if (
            drive_limits.current_limit > 0.0f &&
            (fabs(i_q_set) > fabs(drive_limits.current_limit))
        ) {
            i_q_set = copysign(drive_limits.current_limit, i_q_set);
        }
        // absolute limit on currents defined by the hardware safe operation region
        if (fabs(i_q_set) > 30.0f) {
            i_q_set = copysign(30.0f, i_q_set);
        }
        if (filters_config.i_q_slew_rate > 0.0f) {
            const float max_i_q_delta = filters_config.i_q_slew_rate * T;
            const float i_q_delta = std::clamp(
                i_q_set - i_q_set_slewed,
                -max_i_q_delta,
                max_i_q_delta
            );
            i_q_set_slewed += i_q_delta;
        } else {
            i_q_set_slewed = i_q_set;
        }

        i_q_error = i_q_set_slewed - I_Q;
        q_response = q_reg.regulation(i_q_error, T, busV);
        V_q = q_response;
        #ifdef FOC_PROFILE
        foc_profile.outer_loop = DWT->CYCCNT - t_start;
        #endif
    }

    limit_norm(&V_d, &V_q, busV);

    #ifdef FOC_PROFILE
    t_start = DWT->CYCCNT;
    #endif
    float v_u = 0, v_v = 0, v_w = 0;
    float dtc_u = 0, dtc_v = 0, dtc_w = 0;

    // inverse dq0 transform on voltages
    abc(elec_angles.s, elec_angles.c, V_d, V_q, &v_u, &v_v, &v_w);
    // space vector modulation
    svm(busV, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);

    DQs[0] = (uint16_t)(float(full_pwm + 1) * dtc_u);
    DQs[1] = (uint16_t)(float(full_pwm + 1) * dtc_v);
    DQs[2] = (uint16_t)(float(full_pwm + 1) * dtc_w);

    set_pwm();
    #ifdef FOC_PROFILE
    foc_profile.pwm = DWT->CYCCNT - t_start;
    #endif

#ifdef FOC_PROFILE
    foc_profile.total = DWT->CYCCNT - start_total;
#endif
}


#endif
#endif
