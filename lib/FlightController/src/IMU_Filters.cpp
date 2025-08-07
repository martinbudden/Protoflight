#include "IMU_Filters.h"
#include <MotorMixerBase.h>
#include <RPM_Filters.h>


IMU_Filters::IMU_Filters(const MotorMixerBase& motorMixer, uint32_t looptimeUs) :
    _motorMixer(motorMixer),
    _looptimeUs(looptimeUs),
    _deltaT(static_cast<float>(looptimeUs) * 0.000001F),
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
    _gyroLPF1[X] = &_filterNull;
    _gyroLPF1[Y] = &_filterNull;
    _gyroLPF1[Z] = &_filterNull;

    // set up gyroLPF2. This is the anti-alias filter and should not be disabled.
    const uint16_t gyro_lpf2_hz = config.gyro_lpf2_hz == 0 ? 500 : config.gyro_lpf2_hz;
    switch (config.gyro_lpf2_type) {
    case config_t::BIQUAD: {
        static constexpr float Q = 0.7071067811865475F; // 1 / sqrt(2)
        _lpf2Biquad[X].initLowPass(gyro_lpf2_hz, _looptimeUs, Q);
        _lpf2Biquad[Y].initLowPass(gyro_lpf2_hz, _looptimeUs, Q);
        _lpf2Biquad[Z].initLowPass(gyro_lpf2_hz, _looptimeUs, Q);
        _gyroLPF2[X] = &_lpf2Biquad[X];
        _gyroLPF2[Y] = &_lpf2Biquad[Y];
        _gyroLPF2[Z] = &_lpf2Biquad[Z];
        break;
    }
    case config_t::PT3:
        // just used PT2 if PT3 specified
        [[fallthrough]];
    case config_t::PT2:
        _lpf2PT2[X].setCutoffFrequency(gyro_lpf2_hz, _deltaT);
        _lpf2PT2[Y].setCutoffFrequency(gyro_lpf2_hz, _deltaT);
        _lpf2PT2[Z].setCutoffFrequency(gyro_lpf2_hz, _deltaT);
        _gyroLPF2[X] = &_lpf2PT2[X];
        _gyroLPF2[Y] = &_lpf2PT2[Y];
        _gyroLPF2[Z] = &_lpf2PT2[Z];
        break;
    case config_t::PT1:
        [[fallthrough]];
    default:
        _lpf2PT1[X].setCutoffFrequency(gyro_lpf2_hz, _deltaT);
        _lpf2PT1[Y].setCutoffFrequency(gyro_lpf2_hz, _deltaT);
        _lpf2PT1[Z].setCutoffFrequency(gyro_lpf2_hz, _deltaT);
        _gyroLPF2[X] = &_lpf2PT1[X];
        _gyroLPF2[Y] = &_lpf2PT1[Y];
        _gyroLPF2[Z] = &_lpf2PT1[Z];
        break;
    }

    // set gyroNotch1
    const uint32_t frequencyNyquist = (1000000 / _looptimeUs) / 2;
    if (config.gyro_notch1_hz == 0 || config.gyro_notch1_hz > frequencyNyquist || config.gyro_notch1_cutoff == 0) {
        _gyroNotch1[X] = &_filterNull;
        _gyroNotch1[Y] = &_filterNull;
        _gyroNotch1[Z] = &_filterNull;
    } else {
        _notch1Biquad[X].setNotchFrequency(config.gyro_notch1_hz, config.gyro_notch1_cutoff);
        _notch1Biquad[Y].setNotchFrequency(config.gyro_notch1_hz, config.gyro_notch1_cutoff);
        _notch1Biquad[Z].setNotchFrequency(config.gyro_notch1_hz, config.gyro_notch1_cutoff);
        _gyroNotch1[X] = &_notch1Biquad[X];
        _gyroNotch1[Y] = &_notch1Biquad[Y];
        _gyroNotch1[Z] = &_notch1Biquad[Z];
    }

    _gyroNotch2[X] = &_filterNull;
    _gyroNotch2[Y] = &_filterNull;
    _gyroNotch2[Z] = &_filterNull;
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

    // apply the lowpass filter
    gyroRPS.x = _gyroLPF2[X]->filterVirtual(gyroRPS.x);
    gyroRPS.y = _gyroLPF2[Y]->filterVirtual(gyroRPS.y);
    gyroRPS.z = _gyroLPF2[Z]->filterVirtual(gyroRPS.z);

    // apply the notch filter
    gyroRPS.x = _gyroNotch1[X]->filterVirtual(gyroRPS.x);
    gyroRPS.y = _gyroNotch1[Y]->filterVirtual(gyroRPS.y);
    gyroRPS.z = _gyroNotch1[Z]->filterVirtual(gyroRPS.z);

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
