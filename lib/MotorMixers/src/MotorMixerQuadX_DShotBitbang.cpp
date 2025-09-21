#include "DynamicIdleController.h"
#include "MotorMixerQuadX_DShotBitbang.h"

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
        const float throttleIncrease = _dynamicIdleController.calculateSpeedIncrease(calculateSlowestMotorHz(), deltaT);
        const float throttle = commands.throttle + throttleIncrease;
        _throttleCommand = throttle;

        // calculate the "mix" for the QuadX motor configuration
        _motorOutputs[M0] = -commands.roll - commands.pitch - commands.yaw + throttle;
        _motorOutputs[M1] = -commands.roll + commands.pitch + commands.yaw + throttle;
        _motorOutputs[M2] =  commands.roll - commands.pitch + commands.yaw + throttle;
        _motorOutputs[M3] =  commands.roll + commands.pitch - commands.yaw + throttle;
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
        _throttleCommand = commands.throttle;
    }

    // convert motor output to DShot range [47, 2047]
    _escDShot.outputToMotors(
        static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M0], _motorOutputMin, 1.0F)) + 47),
        static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M1], _motorOutputMin, 1.0F)) + 47),
        static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M2], _motorOutputMin, 1.0F)) + 47),
        static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M3], _motorOutputMin, 1.0F)) + 47)
    );

    // read the motor RPM, used to set the RPM filters
    _motorFrequenciesHz[0] = static_cast<float>(_escDShot.getMotorERPM(0))*_eRPMtoHz;
    _motorFrequenciesHz[1] = static_cast<float>(_escDShot.getMotorERPM(1))*_eRPMtoHz;
    _motorFrequenciesHz[2] = static_cast<float>(_escDShot.getMotorERPM(2))*_eRPMtoHz;
    _motorFrequenciesHz[3] = static_cast<float>(_escDShot.getMotorERPM(3))*_eRPMtoHz;
}
