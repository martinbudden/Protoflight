#pragma once

#include <Filters.h>
#include <array>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#endif
#if defined(FRAMEWORK_RPI_PICO)
#include <pico/critical_section.h>
#include <pico/mutex.h>
#endif

struct  xyz_t;

/*!
There are up to 6 RPM filters for each motor.

There are three filters, one for each of X, Y, and Z at the FUNDAMENTAL frequency.

There are optionally three more filters, one for each of X, Y, and Z at a HARMONIC frequency.

If a harmonic frequency used, it can be set to be either the SECOND HARMONIC (ie twice the FUNDAMENTAL frequency)
or the THIRD HARMONIC (ie 3 times the FUNDAMENTAL frequency).

Generally speaking, the SECOND HARMONIC is used for 2-bladed propellors, and the THIRD HARMONIC is used
for 3-bladed propellors.
*/
class RPM_Filter {
public:
    enum { FUNDAMENTAL = 0, HARMONIC = 1, MAX_HARMONICS_COUNT = 2 };
    enum { MAX_MOTOR_COUNT = 4 };
    enum { X = 0, Y = 1, Z = 2, AXIS_COUNT = 3 };
    enum { USE_FUNDAMENTAL_ONLY = 0, USE_FUNDAMENTAL_AND_SECOND_HARMONIC = 1, USE_FUNDAMENTAL_AND_THIRD_HARMONIC = 2 };
public:
    RPM_Filter(size_t motorCount, uint32_t looptimeUs) : _motorCount(motorCount), _looptimeUs(looptimeUs) {}
    void init(uint32_t harmonicToUse, float Q);
    void setFrequency(size_t motorIndex, float frequencyHz);
    void filter(xyz_t& input, size_t motorIndex);
    size_t getMotorCount() const { return _motorCount; }

    static inline float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
    static inline float sinOrder5Unchecked(float x) {
        // assumes x is in the range [0, PI/2]
        // https://breder.org/sine-polynomial, for degree 5 approximation
        static constexpr float c1 =  0.99977140751539240F;
        static constexpr float c3 = -0.16582704279148017F;
        static constexpr float c5 =  0.0075742477643552034F;
        const float x2 = x*x;
        return x*(c1 + x2*(c3 + x2*c5));
    }
    static inline float sinOrder5(float x) {
        // we know x is in the rang [0, PI], having calculated it
        if (x > 0.5F*M_PI_F) { // get x into range [0, PI/2]
            x =  M_PI_F - x;
        }
        return sinOrder5Unchecked(x);
    }
    static inline float sinOrder7(float x) {
        // we know x is in the rang [0, PI], having calculated it
        if (x > 0.5F*M_PI_F) { // get x into range [0, PI/2]
            x =  M_PI_F - x;
        }
        // https://joelkp.frama.io/tech/modified-taylor.html, for degree 7 approximation
        static constexpr float c1 =  0.99999661599039058046F;
        static constexpr float c3 = -0.99988967477352697077F / 6.0F;
        static constexpr float c5 =  0.99675900242734494228F / 120.0F;
        static constexpr float c7 = -0.92552840500237565369F / 5040.0F;
        const float x2 = x*x;
        return x*(c1 + x2*(c3 + x2*(c5 + x2*c7)));
    }
    static inline float cosOrder5(float x) {
        // we know x is in the rang [0, PI], having calculated it
        // shift x into the range [0, PI/2], for calculating via sin()
        return x < 0.5F*M_PI_F ? sinOrder5Unchecked(0.5F*M_PI_F - x) : -sinOrder5Unchecked(x - 0.5F*M_PI_F);
    }
    static inline float cosOrder7(float x) {
        // x is in the range [0, PI], ensure it stays in that range when shifted for calculating cos
        return x < 0.5F*M_PI_F ? sinOrder7(0.5F*M_PI_F - x) : -sinOrder7(x - 0.5F*M_PI_F);
    }
public:
    static constexpr float M_PI_F = 3.141592653589793F;
private:
    size_t _motorCount;
    uint32_t _looptimeUs;
    uint32_t _harmonicToUse {USE_FUNDAMENTAL_ONLY};
    uint32_t _filterHarmonic {};
    std::array<float, MAX_HARMONICS_COUNT> _weights = { 1.0F, 1.0F };
    float _minFrequencyHz { 100.0F };
    float _maxFrequencyHz {};
    float _halfOfMaxFrequencyHz {};
    float _thirdOfMaxFrequencyHz {};
    float _fadeRangeHz { 50.0F };
    float _Q { 0.0F };
    BiquadFilter _filters[MAX_MOTOR_COUNT][MAX_HARMONICS_COUNT][AXIS_COUNT];
#if defined(USE_FREERTOS)
#if false
    mutable portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;
    // taskENTER_CRITICAL disables interrupts. This also means context switches are prevented. 
    inline void LOCK_FILTERS() const { taskENTER_CRITICAL(&_spinlock); }
    inline void UNLOCK_FILTERS() const { taskEXIT_CRITICAL(&_spinlock); }
#else
    // vTaskSuspendAll suspends the scheduler. This prevents a context switch from occurring but leaves interrupts enabled.
    inline void LOCK_FILTERS() const { vTaskSuspendAll(); }
    inline void UNLOCK_FILTERS() const { xTaskResumeAll(); }
#endif
#elif defined(FRAMEWORK_RPI_PICO)
    mutable mutex_t _mutex {};
    inline void LOCK_FILTERS() const { mutex_enter_blocking(&_mutex); }
    inline void UNLOCK_FILTERS() const { mutex_exit(&_mutex); }
#else
    inline void LOCK_FILTERS() const {}
    inline void UNLOCK_FILTERS() const {}
#endif
};
