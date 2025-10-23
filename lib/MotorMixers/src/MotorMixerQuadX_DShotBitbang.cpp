#include "DynamicIdleController.h"
#include "Mixers.h"
#include "MotorMixerQuadX_DShotBitbang.h"

#include <RPM_Filters.h>

#include <cmath>


MotorMixerQuadX_DShotBitbang::MotorMixerQuadX_DShotBitbang(uint32_t taskIntervalMicroseconds, Debug& debug, const stm32_motor_pins_t& pins, RPM_Filters& rpmFilters) :
    MotorMixerQuadBase(debug),
    _dynamicIdleController(taskIntervalMicroseconds, debug),
    _rpmFilters(rpmFilters)
{
    (void)pins; // !!TODO: set pins
    static constexpr float SECONDS_PER_MINUTE = 60.0F;
    _eRPMtoHz = 2.0F * (100.0F / SECONDS_PER_MINUTE) / static_cast<float>(_motorPoleCount);
}

float MotorMixerQuadX_DShotBitbang::calculateSlowestMotorHz() const
{
    float slowestMotorHz = _motorFrequenciesHz[M0];
    float motorHz = _motorFrequenciesHz[M1];
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    motorHz = _motorFrequenciesHz[M2];
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    motorHz = _motorFrequenciesHz[M3];
    if (motorHz < slowestMotorHz) {
        slowestMotorHz = motorHz;
    }
    return slowestMotorHz;
}

const DynamicIdleController* MotorMixerQuadX_DShotBitbang::getDynamicIdleController() const
{
    return &_dynamicIdleController;
}

void MotorMixerQuadX_DShotBitbang::setDynamicIdlerControllerConfig(const DynamicIdleController::config_t& config)
{
    _dynamicIdleController.setConfig(config);
}

void MotorMixerQuadX_DShotBitbang::outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)tickCount;

    if (motorsIsOn()) {
        const float throttleIncrease = (_dynamicIdleController.getMinimumAllowedMotorHz() == 0.0F) ? 0.0F : _dynamicIdleController.calculateSpeedIncrease(calculateSlowestMotorHz(), deltaT);
        commands.throttle += throttleIncrease;
        // set the throttle to value returned by the mixer
        commands.throttle = mixQuadX(_motorOutputs, commands, _motorOutputMin);
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }

    // convert motor output to DShot range [47, 2047]
    _escDShot.outputToMotors(
        static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M0], _motorOutputMin, 1.0F)) + 47),
        static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M1], _motorOutputMin, 1.0F)) + 47),
        static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M2], _motorOutputMin, 1.0F)) + 47),
        static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M3], _motorOutputMin, 1.0F)) + 47)
    );

    // read the motor RPMs
    _motorFrequenciesHz[M0] = static_cast<float>(_escDShot.getMotorERPM(M0))*_eRPMtoHz;
    _motorFrequenciesHz[M1] = static_cast<float>(_escDShot.getMotorERPM(M1))*_eRPMtoHz;
    _motorFrequenciesHz[M2] = static_cast<float>(_escDShot.getMotorERPM(M2))*_eRPMtoHz;
    _motorFrequenciesHz[M3] = static_cast<float>(_escDShot.getMotorERPM(M3))*_eRPMtoHz;

    _rpmFilters.setFrequencyHz(M0, _motorFrequenciesHz[M0]);
    _rpmFilters.setFrequencyHz(M1, _motorFrequenciesHz[M1]);
    _rpmFilters.setFrequencyHz(M1, _motorFrequenciesHz[M2]);
    _rpmFilters.setFrequencyHz(M3, _motorFrequenciesHz[M3]);

}

void MotorMixerQuadX_DShotBitbang::rpmFilterIterationStep()
{
    // Perform an rpmFilter iteration step for each motor
    // Note that _rpmFilters.iterationStep is an expensive calculation and runs off a state machine, setting one motor harmonic per iteration
    // so we want to call it even if we do not write to the motors
    _rpmFilters.iterationStep();
    _rpmFilters.iterationStep();
    _rpmFilters.iterationStep();
    _rpmFilters.iterationStep();
}
