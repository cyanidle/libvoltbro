#pragma once

#include <utility>

#ifdef ARM_MATH_CM4
#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif
#else
#include "math.h"
#endif

#include "stdint.h"

typedef uint16_t pin;
typedef uint32_t pwm_channel;
typedef uint32_t dac_channel;
typedef uint32_t tim_register;
typedef uint16_t encoder_data;

typedef uint32_t millis;
typedef uint64_t micros;

#define pi2 6.28318530718f

#define FORCE_INLINE __attribute__((always_inline)) inline
#define arm_atomic(T) alignas(T) T

static FORCE_INLINE int64_t subtract_64(uint64_t first, uint64_t second) {
    uint64_t abs_diff = (first > second) ? (first - second): (second - first);
    return (first > second) ? (int64_t)abs_diff : -(int64_t)abs_diff;
}

#if defined(STM32G)
#define CRITICAL_SECTION(code_blk)          \
    uint32_t primask_bit = __get_PRIMASK(); \
    __disable_irq();                        \
    code_blk                                \
    __set_PRIMASK(primask_bit);

// TODO: add optional warning message?
#define HAL_IMPORTANT(command) \
    if ((command) != HAL_OK) { \
        Error_Handler();       \
    }
#endif

#define EACH_N(_value, _counter, N, code_blk) \
    if ((_value - _counter) >= (N)) {         \
        code_blk                              \
        _counter = _value;                    \
    }

#define EACH_N_MICROS(_value, _counter, N, code_blk)         \
    int64_t diff_##_counter = subtract_64(_value, _counter); \
    if (diff_##_counter >= (int64_t)N) {                     \
        code_blk                                             \
        _counter = _value;                                   \
    }

#define EPS 1e-10

inline uint32_t dac_value(double dac_voltage) {
    return floor(4095 * dac_voltage / 3.3);
}

inline bool is_close(float x, float y) {
    return fabsf(x - y) < EPS;
}

inline bool is_close(float x, float y, float tolerance) {
    return fabsf(x - y) < tolerance;
}

template <class T>
class ReservedObject {
private:
    unsigned char buffer[sizeof(T)];
    T* obj;
public:
    template <class... Args>
    void create(Args&&... args) {
        obj = new (buffer) T(std::forward<Args>(args)...);
    }

    T* pointer() {
        return obj;
    }
    T* operator->() {
        return obj;
    }

    ~ReservedObject() {
        obj->~T();
    }
};

constexpr micros MICROS_S = 1'000'000;
constexpr micros MICROS_0_1S = 100'000;
constexpr micros MICROS_0_01S = 10'000;
constexpr micros MICROS_0_001S = 1000;

template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
    return static_cast<typename std::underlying_type<E>::type>(e);
}

constexpr uint16_t CYCLES_100NS_160Mhz = 16;

__attribute__((optimize("O1"))) static inline void delay_cpu_cycles(uint16_t cycles) {
    /* Reference:
     https://developer.arm.com/documentation/ddi0439/b/Programmers-Model/Instruction-set-summary/Cortex-M4-instructions?lang=en
     *
     * // 6 тактов на (cycles - 8) / 5
       sub     r3, r0, #5         // 1 такт
       ldr     r2, .L6            // 2 такта
       smull   r1, r2, r3, r2     // 1 такт
       asr     r3, r3, #31        // 1 такт
       rsb     r3, r3, r2, asr #1 // 1 такт
     *
     * // 2 такта на стартовую проверку
       ands    r3, r3, #255       // 1 такт
       bxeq    lr                 // 1 такт ("Conditional branch completes in a single cycle if the
     branch is not taken.")
     *
     * // ~5 тактов на цикл
       .L3:
       nop                       // 1 такт
       sub     r3, r3, #1        // 1 такт
       ands    r3, r3, #255      // 1 такт
       bne     .L3               // 1 + 1-3 такта, в среднем 2(3?)
     *
     * Всего 5 тактов на цикл + 8 в начале.
     */
    constexpr uint8_t SETUP_CYCLES = 8;
    constexpr uint8_t LOOP_CYCLES = 5;

    uint8_t real_cycles = (cycles - SETUP_CYCLES) / LOOP_CYCLES;
    while (real_cycles--) {
        // NOLINTBEGIN(hicpp-no-assembler)
        __asm__("nop");
        // NOLINTEND(hicpp-no-assembler)
    }
}
