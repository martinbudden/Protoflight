#include "IMU_Filters.h"
#include <MotorMixerBase.h>


IMU_Filters::IMU_Filters(const MotorMixerBase& motorMixer, uint32_t looptimeUs) :
    _motorMixer(motorMixer),
    _looptimeUs(looptimeUs),
    _motorCount(motorMixer.getMotorCount()),
    _rpmFilter(motorMixer.getMotorCount(), looptimeUs)
{
    constexpr float defaultQ = 5.0F;
    _rpmFilter.init(RPM_Filter::USE_FUNDAMENTAL_ONLY, defaultQ);
}

void IMU_Filters::init(float Q)
{
    _rpmFilter.init(RPM_Filter::USE_FUNDAMENTAL_ONLY, Q);
}

void IMU_Filters::setFilters(const filters_t& filters)
{
    _filters = filters;
}

/*!
This is called from withing AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.

This takes, on average, about 26 microseconds to execute on a 240MHz ESP32 S3, with USE_THIRD_HARMONIC.
Execution time varies, however, and may take up to about 50 microseconds.
*/
void IMU_Filters::setFilters()
{
    _rpmFilter.setFrequency(_motorIndex, _motorMixer.getMotorFrequencyHz(_motorIndex));
    ++_motorIndex;
    if (_motorIndex==_motorCount) {
        _motorIndex = 0;
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
    if (_rpmFilter.getMotorCount() == 4) {
        // unwind loop if there are only 4 motors
        _rpmFilter.filter(gyroRPS, 0);
        _rpmFilter.filter(gyroRPS, 1);
        _rpmFilter.filter(gyroRPS, 2);
        _rpmFilter.filter(gyroRPS, 3);
    } else {
        for (size_t motorIndex = 0; motorIndex < _rpmFilter.getMotorCount(); ++motorIndex) {
            _rpmFilter.filter(gyroRPS, motorIndex);
        }
    }
}
