#include "IMU_Filters.h"
#include <motor_mixer_base.h>

#if (__cplusplus >= 202002L)
#include <ranges>
#endif


IMU_Filters::IMU_Filters(float looptimeSeconds) :
    _looptimeSeconds(looptimeSeconds)
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    ,_dynamicNotchFilter(looptimeSeconds)
#endif
{
}

void IMU_Filters::setConfig(const imu_filters_config_t& config)
{
    _config = config;

    // set up accLPF
    if (config.acc_lpf_hz == 0) {
        _accLPF.set_to_passthrough();
    } else {
        _accLPF.set_cutoff_frequency(config.acc_lpf_hz, _looptimeSeconds);
    }

    // set up gyroLPF1.
    const uint16_t gyro_lpf1_hz = config.gyro_lpf1_hz == 0 ? 500 : config.gyro_lpf1_hz;
    switch (config.gyro_lpf1_type) {
    case imu_filters_config_t::NOT_SET:
        _gyroLPF1_PT1.set_to_passthrough();
        _gyroLPF1 = &_gyroLPF1_PT1;
        break;
    case imu_filters_config_t::BIQUAD:
#if defined(USE_GYRO_FILTERS_EXTENDED)
        {static constexpr float Q = 0.7071067811865475F; // 1 / sqrt(2)
        _gyroLPF1_Biquad.init_lowpass(gyro_lpf1_hz, _looptimeSeconds, Q);}
        _gyroLPF1 = &_gyroLPF1_Biquad;
        break;
#else
        [[fallthrough]];
#endif
    case imu_filters_config_t::PT3:
        // just use PT2 if PT3 specified
        [[fallthrough]];
    case imu_filters_config_t::PT2:
#if defined(USE_GYRO_FILTERS_EXTENDED)
        _gyroLPF1_PT2.set_cutoff_frequency(gyro_lpf1_hz, _looptimeSeconds);
        _gyroLPF1 = &_gyroLPF1_PT2;
        break;
#else
        [[fallthrough]];
#endif
    case imu_filters_config_t::PT1:
        [[fallthrough]];
    default:
        _gyroLPF1_PT1.set_cutoff_frequency(gyro_lpf1_hz, _looptimeSeconds);
        _gyroLPF1 = &_gyroLPF1_PT1;
        break;
    }

    // set up gyroLPF2. This is the anti-alias filter can not be disabled.
    const uint16_t gyro_lpf2_hz = config.gyro_lpf2_hz == 0 ? 250 : config.gyro_lpf2_hz;
    _gyroLPF2.set_cutoff_frequency(gyro_lpf2_hz, _looptimeSeconds);

    // setup the notch filters
    const uint32_t frequencyNyquist = static_cast<uint32_t>(std::lroundf((1.0F/_looptimeSeconds)/2.0F));
    if (config.gyro_notch1_hz == 0 || config.gyro_notch1_hz > frequencyNyquist || config.gyro_notch1_cutoff == 0) {
        _useGyroNotch1 = false;
    } else {
        _useGyroNotch1 = true;
        _gyroNotch1.set_notch_frequency(config.gyro_notch1_hz, config.gyro_notch1_cutoff);
    }
    if (config.gyro_notch2_hz == 0 || config.gyro_notch2_hz > frequencyNyquist || config.gyro_notch2_cutoff == 0) {
        _useGyroNotch2 = false;
    } else {
        _useGyroNotch2 = true;
        _gyroNotch2.set_notch_frequency(config.gyro_notch2_hz, config.gyro_notch2_cutoff);
    }
}

void IMU_Filters::set_dynamic_notch_filter_config(const dynamic_notch_filter_config_t& config)
{
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    _dynamicNotchFilter.setConfig(config);
#else
    (void)config;
#endif
}

/*!
This is called from within AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.
*/
void IMU_Filters::filter(xyz_t& gyro_rps, xyz_t& acc, float deltaT, Debug& debug)
{
    (void)deltaT;

    // apply the acc lowpass filter
    acc = _accLPF.filter(acc);

    // apply the gyro lowpass filters
    if (_gyroLPF1) {
        // call filterVirtual(), since type of filter is variable
        gyro_rps = _gyroLPF1->filter_virtual(gyro_rps);
    }
    // gyroLPF2 is always applied
    gyro_rps = _gyroLPF2.filter(gyro_rps);

    // apply the notch filters
    if (_useGyroNotch1) {
        gyro_rps = _gyroNotch1.filter(gyro_rps);
    }
    if (_useGyroNotch2) {
        gyro_rps = _gyroNotch2.filter(gyro_rps);
    }
#if defined(USE_RPM_FILTERS)
    if (_rpmFilters->isActive()) {
        const size_t motorCount = _rpmFilters->get_motor_count();
        // apply the RPM filters
        if (motorCount == 4) {
            _rpmFilters->filter(gyro_rps, 0);
            _rpmFilters->filter(gyro_rps, 1);
            _rpmFilters->filter(gyro_rps, 2);
            _rpmFilters->filter(gyro_rps, 3);
        } else {
#if (__cplusplus >= 202002L)
            for (auto ii : std::views::iota(size_t{0}, motorCount)) {
#else
            for (size_t ii = 0; ii < motorCount; ++ii) {
#endif
               _rpmFilters->filter(gyro_rps, ii);
            }
        }
    }
#endif
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    if (_dynamicNotchFilter.isActive()) {
        // update the notch frequencies using a Fourier transform
        // uses a state machine to perform one iteration each time it is called
        _dynamicNotchFilter.updateNotchFrequencies(debug);
        // push the filtered gyro_rps value to the dynamicNotchFilter after all the other filtration,
        // so it can find the remaining noisy frequencies
        _dynamicNotchFilter.push(gyro_rps);
        // filter uses a state machine to perform one iteration each time it is called
        _dynamicNotchFilter.filter(gyro_rps);
    }
#else
    (void)debug;
#endif
}
