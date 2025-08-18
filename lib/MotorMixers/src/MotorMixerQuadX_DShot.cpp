#include "DynamicIdleController.h"
#include "MotorMixerQuadX_DShot.h"

#include <RPM_Filters.h>


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
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }

    // and finally output to the motors, reading the motor RPM to set the RPM filters
    // motor outputs are converted to DShot range [47,2047]
    _motorBR.write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[MOTOR_BR], _motorOutputMin, 1.0F)) + 47)),
    _motorBR.read();
    _rpmFilters.setFrequencyHz(MOTOR_BR, _motorBR.getMotorHz());

    _motorFR.write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[MOTOR_FR], _motorOutputMin, 1.0F)) + 47)),
    _motorFR.read();
    _rpmFilters.setFrequencyHz(MOTOR_FR, _motorBR.getMotorHz());

    _motorBL.write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[MOTOR_BL], _motorOutputMin, 1.0F)) + 47)),
    _motorBL.read();
    _rpmFilters.setFrequencyHz(MOTOR_BL, _motorBR.getMotorHz());

    _motorFL.write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[MOTOR_FL], _motorOutputMin, 1.0F)) + 47)),
    _motorFL.read();
    _rpmFilters.setFrequencyHz(MOTOR_FL, _motorBR.getMotorHz());
}
