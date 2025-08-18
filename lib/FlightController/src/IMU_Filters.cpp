#include "IMU_Filters.h"
#include <MotorMixerBase.h>
#include <RPM_Filters.h>


IMU_Filters::IMU_Filters(const MotorMixerBase& motorMixer, float looptimeSeconds) :
    _motorMixer(motorMixer),
    _looptimeSeconds(looptimeSeconds),
    _motorCount(motorMixer.getMotorCount())
{
}

void IMU_Filters::setRPM_Filters(RPM_Filters* rpmFilters)
{
    _rpmFilters = rpmFilters;
    constexpr float defaultQ = 5.0F;
    _rpmFilters->init(RPM_Filters::USE_FUNDAMENTAL_ONLY, defaultQ);
    _rpmFilters->setMinimumFrequencyHz(_config.rpm_filter_min_hz);
    //_rpmFilters->setHarmonicToUse(_config.rpm_filter_harmonics);
}

void IMU_Filters::init(float Q)
{
    if (_rpmFilters) {
        _rpmFilters->init(RPM_Filters::USE_FUNDAMENTAL_ONLY, Q);
    }
}

void IMU_Filters::setConfig(const config_t& config)
{
    _config = config;

    // gyro LPF1 currently not used
    _gyroLPF1 = nullptr;

    // set up gyroLPF2. This is the anti-alias filter and should not be disabled.
    const uint16_t gyro_lpf2_hz = config.gyro_lpf2_hz == 0 ? 500 : config.gyro_lpf2_hz;
    switch (config.gyro_lpf2_type) {
    case config_t::BIQUAD: {
        static constexpr float Q = 0.7071067811865475F; // 1 / sqrt(2)
        _lpf2Biquad.initLowPass(gyro_lpf2_hz, _looptimeSeconds, Q);
        _gyroLPF2 = &_lpf2Biquad;
        break;
    }
    case config_t::PT3:
        // just used PT2 if PT3 specified
        [[fallthrough]];
    case config_t::PT2:
        _lpf2PT2.setCutoffFrequency(gyro_lpf2_hz, _looptimeSeconds);
        _gyroLPF2 = &_lpf2PT2;
        break;
    case config_t::PT1:
        [[fallthrough]];
    default:
        _lpf2PT1.setCutoffFrequency(gyro_lpf2_hz, _looptimeSeconds);
        _gyroLPF2 = &_lpf2PT1;
        break;
    }

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
This may be called from withing AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.

This takes, on average, about 26 microseconds to execute on a 240MHz ESP32 S3, with USE_THIRD_HARMONIC.
Execution time varies, however, and may take up to about 50 microseconds.
*/
void IMU_Filters::setFilters()
{
    if (_filterFromAHRS && _rpmFilters) {
        _rpmFilters->setFrequencyHz(_motorIndex, _motorMixer.getMotorFrequencyHz(_motorIndex));
        ++_motorIndex;
        if (_motorIndex==_motorCount) {
            _motorIndex = 0;
        }
    }
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
    // call filterVirtual(), since type of LPF filter is variable
    if (_gyroLPF1) {
        gyroRPS = _gyroLPF1->filterVirtual(gyroRPS);
    }
    if (_gyroLPF2) {
        gyroRPS = _gyroLPF2->filterVirtual(gyroRPS);
    }

    // apply the notch filters
    if (_useGyroNotch1) {
        gyroRPS = _gyroNotch1.filter(gyroRPS);
    }
    if (_useGyroNotch2) {
        gyroRPS = _gyroNotch2.filter(gyroRPS);
    }

    // apply the RPM filters
    if (_rpmFilters) {
        if (_motorCount == 4) {
            // unwind loop if there are only 4 motors
            _rpmFilters->filter(gyroRPS, 0);
            _rpmFilters->filter(gyroRPS, 1);
            _rpmFilters->filter(gyroRPS, 2);
            _rpmFilters->filter(gyroRPS, 3);
        } else {
            for (size_t motorIndex = 0; motorIndex < _rpmFilters->getMotorCount(); ++motorIndex) {
                _rpmFilters->filter(gyroRPS, motorIndex);
            }
        }
    }
}
