#include "DynamicIdleController.h"
#include "MotorMixerQuadX_DShot.h"

#include <RPM_Filters.h>


MotorMixerQuadX_DShot::MotorMixerQuadX_DShot(Debug& debug, const pins_t& pins, RPM_Filters& rpmFilters, DynamicIdleController& dynamicIdleController) :
    MotorMixerQuadX_Base(debug),
    _rpmFilters(rpmFilters),
    _dynamicIdleController(dynamicIdleController)
{
    _motors[M0].init(pins.m0);
    _motors[M0].init(pins.m1);
    _motors[M0].init(pins.m2);
    _motors[M0].init(pins.m3);
}

float MotorMixerQuadX_DShot::calculateSlowestMotorHz() const
{
    float slowestMotorHz = _motors[M0].getMotorHz();
    float motorHz = _motors[M1].getMotorHz();
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    motorHz = _motors[M2].getMotorHz();
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    motorHz = _motors[M3].getMotorHz();
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    return slowestMotorHz;
}

DynamicIdleController* MotorMixerQuadX_DShot::getDynamicIdleController() const
{
    return &_dynamicIdleController;
}

void MotorMixerQuadX_DShot::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)tickCount;

    if (motorsIsOn()) {
        const float throttleIncrease = _dynamicIdleController.calculateSpeedIncrease(calculateSlowestMotorHz(), deltaT);
        const float throttle = commands.throttle + throttleIncrease;
        _throttleCommand = throttle;

        // calculate the "mix" for the QuadX motor configuration
        _motorOutputs[M0] = throttle - commands.roll + commands.pitch + commands.yaw; // back right
        _motorOutputs[M1] = throttle - commands.roll - commands.pitch - commands.yaw; // front right
        _motorOutputs[M2] = throttle + commands.roll + commands.pitch - commands.yaw; // back left
        _motorOutputs[M3] = throttle + commands.roll - commands.pitch + commands.yaw; // front left 
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
        _throttleCommand = commands.throttle;
    }

    // and finally output to the motors, reading the motor RPM to set the RPM filters
    // motor outputs are converted to DShot range [47,2047]
    _motors[M0].write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M0], _motorOutputMin, 1.0F)) + 47)),
    _motors[M0].read();
    _rpmFilters.setFrequencyHz(M0, _motors[M0].getMotorHz());

    _motors[M1].write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M1], _motorOutputMin, 1.0F)) + 47)),
    _motors[M1].read();
    _rpmFilters.setFrequencyHz(M1, _motors[M1].getMotorHz());

    _motors[M2].write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M2], _motorOutputMin, 1.0F)) + 47)),
    _motors[M2].read();
    _rpmFilters.setFrequencyHz(M2, _motors[M2].getMotorHz());

    _motors[M3].write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M3], _motorOutputMin, 1.0F)) + 47)),
    _motors[M3].read();
    _rpmFilters.setFrequencyHz(M3, _motors[M3].getMotorHz());
}
