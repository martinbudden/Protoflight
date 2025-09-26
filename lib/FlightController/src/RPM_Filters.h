#pragma once

#include <FilterTemplates.h>
#include <array>

#if defined(FRAMEWORK_RPI_PICO)

#include <pico/critical_section.h>
#include <pico/mutex.h>

#elif defined(FRAMEWORK_USE_FREERTOS)

#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#endif

#endif

#include <xyz_type.h>


/*!
There are up to 6 RPM filters for each motor.

There are three filters, one for each of X, Y, and Z at the FUNDAMENTAL frequency.

There are optionally three more filters, one for each of X, Y, and Z at a HARMONIC frequency.

If a harmonic frequency used, it can be set to be either the SECOND HARMONIC (ie twice the FUNDAMENTAL frequency)
or the THIRD HARMONIC (ie 3 times the FUNDAMENTAL frequency).

Generally speaking, the SECOND HARMONIC is used for 2-bladed propellors, and the THIRD HARMONIC is used
for 3-bladed propellors.
*/
class RPM_Filters {
public:
    enum { FUNDAMENTAL = 0, HARMONIC = 1, MAX_HARMONICS_COUNT = 2 };
    enum { MAX_MOTOR_COUNT = 4 };
    enum { USE_FUNDAMENTAL_ONLY = 0, USE_FUNDAMENTAL_AND_SECOND_HARMONIC = 1, USE_FUNDAMENTAL_AND_THIRD_HARMONIC = 2 };
public:
    RPM_Filters(size_t motorCount, float looptimeSeconds) : _motorCount(motorCount), _looptimeSeconds(looptimeSeconds) {}
    void init(uint32_t harmonicToUse, float Q);
    void setHarmonicToUse(uint8_t harmonicToUse) {_harmonicToUse = harmonicToUse; }
    void setMinimumFrequencyHz(float minFrequencyHz) { _minFrequencyHz = minFrequencyHz; }
    void setFrequencyHz(size_t motorIndex, float frequencyHz); // called from the motor mixer
    void filter(xyz_t& input, size_t motorIndex);
    size_t getMotorCount() const { return _motorCount; }

    static inline float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
public:
    static constexpr float M_PI_F = 3.141592653589793F;
private:
    size_t _motorCount;
    float _looptimeSeconds;
    uint32_t _harmonicToUse {USE_FUNDAMENTAL_ONLY};
    uint32_t _filterHarmonic {};
    std::array<float, MAX_HARMONICS_COUNT> _weights = { 1.0F, 1.0F };
    float _minFrequencyHz { 100.0F };
    float _maxFrequencyHz {};
    float _halfOfMaxFrequencyHz {};
    float _thirdOfMaxFrequencyHz {};
    float _fadeRangeHz { 50.0F };
    float _Q { 0.0F };
    BiquadFilterT<xyz_t> _filters[MAX_MOTOR_COUNT][MAX_HARMONICS_COUNT];
#if defined(FRAMEWORK_RPI_PICO)
    mutable mutex_t _mutex {};
    inline void LOCK_FILTERS() const { mutex_enter_blocking(&_mutex); }
    inline void UNLOCK_FILTERS() const { mutex_exit(&_mutex); }
#elif defined(FRAMEWORK_USE_FREERTOS)
    // vTaskSuspendAll suspends the scheduler. This prevents a context switch from occurring but leaves interrupts enabled.
    inline void LOCK_FILTERS() const { vTaskSuspendAll(); }
    inline void UNLOCK_FILTERS() const { xTaskResumeAll(); }
#else
    inline void LOCK_FILTERS() const {}
    inline void UNLOCK_FILTERS() const {}
#endif
};
