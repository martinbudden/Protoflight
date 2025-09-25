#include "DynamicIdleController.h"
#include "Mixers.h"
#include "MotorMixerQuadX_DShot.h"

#include <RPM_Filters.h>


MotorMixerQuadX_DShot::MotorMixerQuadX_DShot(Debug& debug, const motor_pins_t& pins, RPM_Filters& rpmFilters, DynamicIdleController& dynamicIdleController) :
    MotorMixerQuadBase(debug),
    _rpmFilters(rpmFilters),
    _dynamicIdleController(dynamicIdleController)
{
    _motors[M0].init(pins.m0);
    _motors[M1].init(pins.m1);
    _motors[M2].init(pins.m2);
    _motors[M3].init(pins.m3);
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

void MotorMixerQuadX_DShot::outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)tickCount;

    if (motorsIsOn()) {
        const float throttleIncrease = _dynamicIdleController.getMinimumAllowedMotorHz() == 0.0F ? 0.0F : _dynamicIdleController.calculateSpeedIncrease(calculateSlowestMotorHz(), deltaT);
        commands.throttle += throttleIncrease;
        // set the throttle to value returned by the mixer
        commands.throttle = mixQuadX(_motorOutputs, commands);
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
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
