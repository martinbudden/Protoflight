#pragma once

#include <FilterTemplates.h>
#include <Filters.h>
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
    enum { RPM_FILTER_HARMONICS_COUNT = 3 };
    struct config_t {
        uint8_t  rpm_filter_harmonics;      // number of harmonics, zero means filters off
        uint8_t  rpm_filter_weights[RPM_FILTER_HARMONICS_COUNT];    // weight as a percentage for each harmonic
        uint8_t  rpm_filter_min_hz;         // minimum notch frequency for fundamental harmonic
        uint16_t rpm_filter_fade_range_hz;  // range in which notch filters fade down to minHz
        uint16_t rpm_filter_q;              // q of the notch filters
        uint16_t rpm_filter_lpf_hz;         // LPF cutoff (from motor rpm converted to Hz)
    };
public:
    enum { FUNDAMENTAL = 0, SECOND_HARMONIC = 1, THIRD_HARMONIC = 2 };
    enum { MAX_MOTOR_COUNT = 4 };
public:
    RPM_Filters(size_t motorCount, float looptimeSeconds) : _motorCount(motorCount), _looptimeSeconds(looptimeSeconds) {}
    void setConfig(const config_t& config);
    const config_t& getConfig() const { return _config; }
    void setFrequencyHz(size_t motorIndex, float frequencyHz); // called from the motor mixer
    void filter(xyz_t& input, size_t motorIndex);
    size_t getMotorCount() const { return _motorCount; }

    static inline float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
public:
    static constexpr float M_PI_F = 3.141592653589793F;
private:
    size_t _motorCount;
    float _looptimeSeconds;
    // computation data so setFrequencyHz() can be run as a state machine
    enum state_e { STATE_FUNDAMENTAL, STATE_SECOND_HARMONIC, STATE_THIRD_HARMONIC };
    struct state_t {
        state_e state;
        float frequencyHzUnclipped;
        float weightMultiplier;
        float sinOmega;
        float two_cosOmega;
    };
    state_t _state  { STATE_FUNDAMENTAL, 0.0F, 0.0F, 0.0F, 0.0F };

    std::array<float, RPM_FILTER_HARMONICS_COUNT> _weights {};
    float _minFrequencyHz { 100.0F };
    float _maxFrequencyHz {};
    float _halfOfMaxFrequencyHz {};
    float _thirdOfMaxFrequencyHz {};
    float _fadeRangeHz { 50.0F };
    float _Q { 0.0F };
    BiquadFilterT<xyz_t> _filters[MAX_MOTOR_COUNT][RPM_FILTER_HARMONICS_COUNT];
    std::array<PowerTransferFilter1, MAX_MOTOR_COUNT> _motorRPM_Filters {};
    const config_t _config {}; //!< const once has been set in setConfig
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
