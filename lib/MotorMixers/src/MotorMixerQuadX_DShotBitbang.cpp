#include "DynamicIdleController.h"
#include "MotorMixerQuadX_DShotBitbang.h"

#include <Debug.h>
#include <Filters.h>
#include <RPM_Filters.h>
#include <algorithm>
#include <cmath>


MotorMixerQuadX_DShotBitbang::MotorMixerQuadX_DShotBitbang(Debug& debug, const pins_t& pins, RPM_Filters& rpmFilters, DynamicIdleController& dynamicIdleController) :
    MotorMixerQuadX_Base(debug),
    _rpmFilters(rpmFilters),
    _dynamicIdleController(dynamicIdleController)
{
    (void)pins; // !!TODO: set pins
    constexpr float SECONDS_PER_MINUTE = 60.0F;
    _eRPMtoHz = 2.0F * (100.0F / SECONDS_PER_MINUTE) / static_cast<float>(_motorPoleCount);
}

float MotorMixerQuadX_DShotBitbang::calculateSlowestMotorHz() const
{
    float slowestMotorHz = _motorFrequenciesHz[0];
    float motorHz = _motorFrequenciesHz[1];
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    motorHz = _motorFrequenciesHz[2];
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    motorHz = _motorFrequenciesHz[3];
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    return slowestMotorHz;
}

DynamicIdleController* MotorMixerQuadX_DShotBitbang::getDynamicIdleController() const
{
    return &_dynamicIdleController;
}

void MotorMixerQuadX_DShotBitbang::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
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
    _escBitbang.outputToMotors(
        static_cast<uint16_t>(_motorOutputs[MOTOR_BR]),
        static_cast<uint16_t>(_motorOutputs[MOTOR_FR]),
        static_cast<uint16_t>(_motorOutputs[MOTOR_BL]),
        static_cast<uint16_t>(_motorOutputs[MOTOR_FL])
    );
    _motorFrequenciesHz[0] = static_cast<float>(_escBitbang.getMotorERPM(0))*_eRPMtoHz;
    _motorFrequenciesHz[1] = static_cast<float>(_escBitbang.getMotorERPM(1))*_eRPMtoHz;
    _motorFrequenciesHz[2] = static_cast<float>(_escBitbang.getMotorERPM(2))*_eRPMtoHz;
    _motorFrequenciesHz[3] = static_cast<float>(_escBitbang.getMotorERPM(3))*_eRPMtoHz;
}
