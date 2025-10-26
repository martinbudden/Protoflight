#include "IMU_Filters.h"
#include "RPM_Filters.h"
#include <MotorMixerBase.h>


IMU_Filters::IMU_Filters(size_t motorCount, Debug& debug, float looptimeSeconds) :
    _debug(debug),
    _looptimeSeconds(looptimeSeconds),
    _motorCount(motorCount)
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    ,_dynamicNotchFilter(debug, looptimeSeconds)
#endif
#if defined(USE_RPM_FILTERS)
    ,_rpmFilters(motorCount, looptimeSeconds)
#endif
{
}

void IMU_Filters::setConfig(const config_t& config)
{
    _config = config;

    // set up gyroLPF1.
    const uint16_t gyro_lpf1_hz = config.gyro_lpf1_hz == 0 ? 500 : config.gyro_lpf1_hz;
    switch (config.gyro_lpf1_type) {
    case config_t::BIQUAD: {
        static constexpr float Q = 0.7071067811865475F; // 1 / sqrt(2)
        _gyroLPF1Biquad.initLowPass(gyro_lpf1_hz, _looptimeSeconds, Q);
        _gyroLPF1 = &_gyroLPF1Biquad;
        break;
    }
    case config_t::NOT_SET:
        _gyroLPF1PT1.setToPassthrough();
        _gyroLPF1 = &_gyroLPF1PT1;
        break;
    case config_t::PT3:
        // just use PT2 if PT3 specified
        [[fallthrough]];
    case config_t::PT2:
        _gyroLPF1PT2.setCutoffFrequency(gyro_lpf1_hz, _looptimeSeconds);
        _gyroLPF1 = &_gyroLPF1PT2;
        break;
    case config_t::PT1:
        [[fallthrough]];
    default:
        _gyroLPF1PT1.setCutoffFrequency(gyro_lpf1_hz, _looptimeSeconds);
        _gyroLPF1 = &_gyroLPF1PT1;
        break;
    }

    // set up gyroLPF2. This is the anti-alias filter can not be disabled.
    const uint16_t gyro_lpf2_hz = config.gyro_lpf2_hz == 0 ? 250 : config.gyro_lpf2_hz;
    _gyroLPF2.setCutoffFrequency(gyro_lpf2_hz, _looptimeSeconds);

    // setup the notch filters
    const uint32_t frequencyNyquist = static_cast<uint32_t>(std::lroundf((1.0F/_looptimeSeconds)/2.0F));
    if (config.gyro_notch1_hz == 0 || config.gyro_notch1_hz > frequencyNyquist || config.gyro_notch1_cutoff == 0) {
        _useGyroNotch1 = false;
    } else {
        _useGyroNotch1 = true;
        _gyroNotch1.setNotchFrequency(config.gyro_notch1_hz, config.gyro_notch1_cutoff);
    }
    if (config.gyro_notch2_hz == 0 || config.gyro_notch2_hz > frequencyNyquist || config.gyro_notch2_cutoff == 0) {
        _useGyroNotch2 = false;
    } else {
        _useGyroNotch2 = true;
        _gyroNotch2.setNotchFrequency(config.gyro_notch2_hz, config.gyro_notch2_cutoff);
    }
}

void IMU_Filters::setDynamicNotchFilterConfig(const DynamicNotchFilter::config_t& config)
{
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    _dynamicNotchFilter.setConfig(config);
#else
    (void)config;
#endif
}

void IMU_Filters::setRPM_FiltersConfig(const RPM_Filters::config_t& config)
{
#if defined(USE_RPM_FILTERS)
    _rpmFilters.setConfig(config);
#else
    (void)config;
#endif
}

/*!
This is called from within AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.
*/
void IMU_Filters::filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT)
{
    (void)acc;
    (void)deltaT;

    // apply the lowpass filters
    if (_gyroLPF1) {
        // call filterVirtual(), since type of filter is variable
        gyroRPS = _gyroLPF1->filterVirtual(gyroRPS);
    }
    // gyroLPF2 is always applied
    gyroRPS = _gyroLPF2.filter(gyroRPS);

    // apply the notch filters
    if (_useGyroNotch1) {
        gyroRPS = _gyroNotch1.filter(gyroRPS);
    }
    if (_useGyroNotch2) {
        gyroRPS = _gyroNotch2.filter(gyroRPS);
    }
#if defined(USE_RPM_FILTERS)
    if (_rpmFilters.isActive()) {
        // apply the RPM filters
        if (_motorCount == 4) {
            _rpmFilters.filter(gyroRPS, 0);
            _rpmFilters.filter(gyroRPS, 1);
            _rpmFilters.filter(gyroRPS, 2);
            _rpmFilters.filter(gyroRPS, 3);
        } else {
            for (size_t ii = 0; ii < _motorCount; ++ii) {
               _rpmFilters.filter(gyroRPS, ii);
            }
        }
    }
#endif
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    if (_dynamicNotchFilter.isActive()) {
        // update the notch frequencies using a Fourier transform
        // uses a state machine to perform one iteration each time it is called
        _dynamicNotchFilter.updateNotchFrequencies();
        // push the filtered gyroRPS value to the dynamicNotchFilter after all the other filtration,
        // so it can find the remaining noisy frequencies
        _dynamicNotchFilter.push(gyroRPS);
        // filter uses a state machine to perform one iteration each time it is called
        _dynamicNotchFilter.filter(gyroRPS);
    }
#endif
}
