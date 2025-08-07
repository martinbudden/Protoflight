#include "DynamicIdleController.h"
#include "MotorMixerQuadX_DShot.h"

#include <Debug.h>
#include <Filters.h>
#include <RPM_Filters.h>
#include <algorithm>
#include <cmath>


MotorMixerQuadX_DShot::MotorMixerQuadX_DShot(Debug& debug, const pins_t& pins, RPM_Filters& rpmFilters, DynamicIdleController& dynamicIdleController) :
    MotorMixerQuadX_Base(debug),
    _rpmFilters(rpmFilters),
    _dynamicIdleController(dynamicIdleController)
{
    _motorBR.init(pins.br);
    _motorFR.init(pins.fr);
    _motorBL.init(pins.bl);
    _motorFL.init(pins.fl);
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
        const float speedIncrease = _dynamicIdleController.calculateSpeedIncrease(calculateSlowestMotorHz(), deltaT);
        const float speed = commands.speed + speedIncrease;

        // calculate the "mix" for the QuadX motor configuration
        _motorOutputs[MOTOR_BR] = -commands.roll - commands.pitch - commands.yaw + speed;
        _motorOutputs[MOTOR_FR] = -commands.roll + commands.pitch + commands.yaw + speed;
        _motorOutputs[MOTOR_BL] =  commands.roll - commands.pitch + commands.yaw + speed;
        _motorOutputs[MOTOR_FL] =  commands.roll + commands.pitch - commands.yaw + speed;

        // scale motor output to [0.0F, 1000.0F], which is the range required for DShot
        _motorOutputs[MOTOR_BR] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_BR], _motorOutputMin, 1.0F));
        _motorOutputs[MOTOR_FR] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_FR], _motorOutputMin, 1.0F));
        _motorOutputs[MOTOR_BL] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_BL], _motorOutputMin, 1.0F));
        _motorOutputs[MOTOR_FL] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_FL], _motorOutputMin, 1.0F));
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }

    // and finally output to the motors, reading the motor RPM to set the RPM filters
    _motorBR.write(static_cast<uint16_t>(_motorOutputs[MOTOR_BR]));
    _motorBR.read();
    _rpmFilters.setFrequencyHz(MOTOR_BR, _motorBR.getMotorHz());

    _motorFR.write(static_cast<uint16_t>(_motorOutputs[MOTOR_FR]));
    _motorFR.read();
    _rpmFilters.setFrequencyHz(MOTOR_FR, _motorBR.getMotorHz());

    _motorBL.write(static_cast<uint16_t>(_motorOutputs[MOTOR_BL]));
    _motorBL.read();
    _rpmFilters.setFrequencyHz(MOTOR_BL, _motorBR.getMotorHz());

    _motorFL.write(static_cast<uint16_t>(_motorOutputs[MOTOR_FL]));
    _motorFL.read();
    _rpmFilters.setFrequencyHz(MOTOR_FL, _motorBR.getMotorHz());
}
