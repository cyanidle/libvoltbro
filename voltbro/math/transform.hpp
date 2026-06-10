#pragma once
#if defined(STM32G4) || defined(STM32_G)

#ifdef __cplusplus
extern "C" {
#endif

#include "math_ops.hpp"

inline void Park(float alpha, float beta, float theta, float* d, float* q) {
    float cosine = arm_cos_f32(theta);
    float sine = arm_sin_f32(theta);
    //*d = alpha*cosine - beta*sine;      //This is a hack - effectively using
    //-beta instead of beta *q = -beta*cosine - alpha*sine;     //I think
    // because I'm using pi as the d axis offset instead of zero, but I need to
    // investigate more.
    *d = alpha * cosine + beta * sine;
    *q = beta * cosine - alpha * sine;
}

inline void InvPark(float d, float q, float theta, float* alpha, float* beta) {
    float cosine = arm_cos_f32(theta);
    float sine = arm_sin_f32(theta);
    *alpha = d * cosine - q * sine;
    *beta = q * cosine + d * sine;
}

inline void Clarke(float a, float b, float* alpha, float* beta) {
    *alpha = a;
    *beta = 0.57735026919f * (a + 2.0f * b);
}

inline void InvClarke(float alpha, float beta, float* a, float* b, float* c) {
    *a = alpha;
    *b = 0.5f * (-alpha + 1.73205080757f * beta);
    *c = 0.5f * (-alpha - 1.73205080757f * beta);
}

inline void dq0(float sf, float cf, float a, float b, float c, float* d, float* q) {
    /// DQ0 Transform ///
    /// Phase current amplitude = lengh of dq vector///
    /// i.e. iq = 1, id = 0, peak phase current of 1///

    // float cf = arm_cos_f32(theta);
    // float sf = arm_sin_f32(theta);

    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b +
                       (-0.86602540378f * sf - .5f * cf) * c);  /// Faster DQ0 Transform
    *q = 0.6666667f *
         (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

inline void abc(float sf, float cf, float d, float q, float* a, float* b, float* c) {
    /// Inverse DQ0 Transform ///
    /// Phase current amplitude = lengh of dq vector///
    /// i.e. iq = 1, id = 0, peak phase current of 1///
    // float cf = arm_cos_f32(theta);
    // float sf = arm_sin_f32(theta);

    *a = cf * d - sf * q;  // Faster Inverse DQ0 transform
    *b = (0.86602540378f * sf - .5f * cf) * d - (-0.86602540378f * cf - .5f * sf) * q;
    *c = (-0.86602540378f * sf - .5f * cf) * d - (0.86602540378f * cf - .5f * sf) * q;
}

inline void svm(float v_bus, float u, float v, float w, float* dtc_u, float* dtc_v, float* dtc_w) {
    /// Space Vector Modulation ///
    /// u,v,w amplitude = v_bus for full modulation depth ///

    #define DTC_MIN 0
    #define DTC_MAX 1

    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w)) / 2.0f;
    *dtc_u = fminf(fmaxf(((u - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);
}

#ifdef __cplusplus
}
#endif

#endif
