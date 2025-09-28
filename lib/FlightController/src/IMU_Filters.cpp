#include "IMU_Filters.h"
#include <MotorMixerBase.h>
#include <RPM_Filters.h>


IMU_Filters::IMU_Filters(const MotorMixerBase& motorMixer, float looptimeSeconds) :
    _motorMixer(motorMixer),
    _looptimeSeconds(looptimeSeconds),
    _motorCount(motorMixer.getMotorCount())
{
}


void IMU_Filters::setConfig(const config_t& config)
{
    const_cast<config_t&>(_config) = config; // NOLINT(cppcoreguidelines-pro-type-const-cast)

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

/*!
Does nothing, since RPM filtering is done in the context of the Flight Controller Task.
This is the place to put the Fast Fourier Transform (FFT) if dynamic notch filters are implemented.
*/
void IMU_Filters::setFilters()
{
}

/*!
This is called from withing AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.

This takes of the order of 15 microseconds to execute on a 240MHz ESP32 S3.
Max time is up to 30 microseconds.
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
}
