#include "imu_filters.h"
#include <motor_mixer_base.h>

#if (__cplusplus >= 202002L)
#include <ranges>
#endif


ImuFilters::ImuFilters(float looptime_seconds) :
    _looptime_seconds(looptime_seconds)
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    ,_dynamic_notch_filter(looptime_seconds)
#endif
{
}

void ImuFilters::set_config(const imu_filters_config_t& config)
{
    _config = config;

    // set up acc_lpf
    if (config.acc_lpf_hz == 0) {
        _acc_lpf.set_to_passthrough();
    } else {
        _acc_lpf.set_cutoff_frequency(config.acc_lpf_hz, _looptime_seconds);
    }

    // set up gyro_lpf1.
    const uint16_t gyro_lpf1_hz = config.gyro_lpf1_hz == 0 ? 500 : config.gyro_lpf1_hz;
    switch (config.gyro_lpf1_type) {
    case imu_filters_config_t::NOT_SET:
        _gyro_lpf1_pt1.set_to_passthrough();
        _gyro_lpf1 = &_gyro_lpf1_pt1;
        break;
    case imu_filters_config_t::BIQUAD:
#if defined(USE_GYRO_FILTERS_EXTENDED)
        {static constexpr float Q = 0.7071067811865475F; // 1 / sqrt(2)
        _gyro_lpf1_biquad.init_lowpass(gyro_lpf1_hz, _looptime_seconds, Q);}
        _gyro_lpf1 = &_gyro_lpf1_biquad;
        break;
#else
        [[fallthrough]];
#endif
    case imu_filters_config_t::PT3:
        // just use PT2 if PT3 specified
        [[fallthrough]];
    case imu_filters_config_t::PT2:
#if defined(USE_GYRO_FILTERS_EXTENDED)
        _gyro_lpf1_pt2.set_cutoff_frequency(gyro_lpf1_hz, _looptime_seconds);
        _gyro_lpf1 = &_gyro_lpf1_pt2;
        break;
#else
        [[fallthrough]];
#endif
    case imu_filters_config_t::PT1:
        [[fallthrough]];
    default:
        _gyro_lpf1_pt1.set_cutoff_frequency(gyro_lpf1_hz, _looptime_seconds);
        _gyro_lpf1 = &_gyro_lpf1_pt1;
        break;
    }

    // set up gyro_lpf2. This is the anti-alias filter can not be disabled.
    const uint16_t gyro_lpf2_hz = config.gyro_lpf2_hz == 0 ? 250 : config.gyro_lpf2_hz;
    _gyro_lpf2.set_cutoff_frequency(gyro_lpf2_hz, _looptime_seconds);

    // setup the notch filters
    const uint32_t frequencyNyquist = static_cast<uint32_t>(std::lroundf((1.0F/_looptime_seconds)/2.0F));
    if (config.gyro_notch1_hz == 0 || config.gyro_notch1_hz > frequencyNyquist || config.gyro_notch1_cutoff == 0) {
        _use_gyro_notch1 = false;
    } else {
        _use_gyro_notch1 = true;
        _gyro_notch1.set_notch_frequency(config.gyro_notch1_hz, config.gyro_notch1_cutoff);
    }
    if (config.gyro_notch2_hz == 0 || config.gyro_notch2_hz > frequencyNyquist || config.gyro_notch2_cutoff == 0) {
        _use_gyro_notch2 = false;
    } else {
        _use_gyro_notch2 = true;
        _gyro_notch2.set_notch_frequency(config.gyro_notch2_hz, config.gyro_notch2_cutoff);
    }
}

void ImuFilters::set_dynamic_notch_filter_config(const dynamic_notch_filter_config_t& config)
{
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    _dynamic_notch_filter.set_config(config);
#else
    (void)config;
#endif
}

/*!
This is called from within AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.
*/
void ImuFilters::filter(xyz_t& acc, xyz_t& gyro_rps, float delta_t, Debug& debug)
{
    (void)delta_t;

    // apply the acc lowpass filter
    acc = _acc_lpf.filter(acc);

    // apply the gyro lowpass filters
    if (_gyro_lpf1) {
        // call filterVirtual(), since type of filter is variable
        gyro_rps = _gyro_lpf1->filter_virtual(gyro_rps);
    }
    // gyro_lpf2 is always applied
    gyro_rps = _gyro_lpf2.filter(gyro_rps);

    // apply the notch filters
    if (_use_gyro_notch1) {
        gyro_rps = _gyro_notch1.filter(gyro_rps);
    }
    if (_use_gyro_notch2) {
        gyro_rps = _gyro_notch2.filter(gyro_rps);
    }
#if defined(USE_RPM_FILTERS)
    if (_rpm_filters->is_active()) {
        const size_t motorCount = _rpm_filters->get_motor_count();
        // apply the RPM filters
        if (motorCount == 4) {
            _rpm_filters->filter(gyro_rps, 0);
            _rpm_filters->filter(gyro_rps, 1);
            _rpm_filters->filter(gyro_rps, 2);
            _rpm_filters->filter(gyro_rps, 3);
        } else {
#if (__cplusplus >= 202002L)
            for (auto ii : std::views::iota(size_t{0}, motorCount)) {
#else
            for (size_t ii = 0; ii < motorCount; ++ii) {
#endif
               _rpm_filters->filter(gyro_rps, ii);
            }
        }
    }
#endif
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    if (_dynamic_notch_filter.is_active()) {
        // update the notch frequencies using a Fourier transform
        // uses a state machine to perform one iteration each time it is called
        _dynamic_notch_filter.update_notch_frequencies(debug);
        // push the filtered gyro_rps value to the dynamic_notch_filter after all the other filtration,
        // so it can find the remaining noisy frequencies
        _dynamic_notch_filter.push(gyro_rps);
        // filter uses a state machine to perform one iteration each time it is called
        _dynamic_notch_filter.filter(gyro_rps);
    }
#else
    (void)debug;
#endif
}
