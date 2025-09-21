#include "DynamicIdleController.h"
#include "MotorMixerQuadX_DShot.h"

#include <RPM_Filters.h>


MotorMixerQuadX_DShot::MotorMixerQuadX_DShot(Debug& debug, const pins_t& pins, RPM_Filters& rpmFilters, DynamicIdleController& dynamicIdleController) :
    MotorMixerQuadX_Base(debug),
    _rpmFilters(rpmFilters),
    _dynamicIdleController(dynamicIdleController)
{
    _motorBR.init(pins.m0);
    _motorFR.init(pins.m1);
    _motorBL.init(pins.m2);
    _motorFL.init(pins.m3);
}

float MotorMixerQuadX_DShot::calculateSlowestMotorHz() const
{
    float slowestMotorHz = _motorBR.getMotorHz();
    float motorHz = _motorFR.getMotorHz();
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    motorHz = _motorBL.getMotorHz();
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    motorHz = _motorFL.getMotorHz();
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
        _motorOutputs[M0] = -commands.roll - commands.pitch - commands.yaw + throttle; // back right
        _motorOutputs[M1] = -commands.roll + commands.pitch + commands.yaw + throttle; // front right
        _motorOutputs[M2] =  commands.roll - commands.pitch + commands.yaw + throttle; // back left
        _motorOutputs[M3] =  commands.roll + commands.pitch - commands.yaw + throttle; // front left 
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
        _throttleCommand = commands.throttle;
    }

    // and finally output to the motors, reading the motor RPM to set the RPM filters
    // motor outputs are converted to DShot range [47,2047]
    _motorBR.write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M0], _motorOutputMin, 1.0F)) + 47)),
    _motorBR.read();
    _rpmFilters.setFrequencyHz(M0, _motorBR.getMotorHz());

    _motorFR.write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M1], _motorOutputMin, 1.0F)) + 47)),
    _motorFR.read();
    _rpmFilters.setFrequencyHz(M1, _motorBR.getMotorHz());

    _motorBL.write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M2], _motorOutputMin, 1.0F)) + 47)),
    _motorBL.read();
    _rpmFilters.setFrequencyHz(M2, _motorBR.getMotorHz());

    _motorFL.write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M3], _motorOutputMin, 1.0F)) + 47)),
    _motorFL.read();
    _rpmFilters.setFrequencyHz(M3, _motorBR.getMotorHz());
}
