#include "DynamicIdleController.h"
#include "Mixers.h"
#include "MotorMixerQuadX_DShot.h"

#include <RPM_Filters.h>


MotorMixerQuadX_DShot::MotorMixerQuadX_DShot(uint32_t taskIntervalMicroseconds, uint32_t outputToMotorsDenominator, Debug& debug, const motor_pins_t& pins, RPM_Filters& rpmFilters) :
    MotorMixerQuadBase(debug),
    _dynamicIdleController(taskIntervalMicroseconds/outputToMotorsDenominator, debug),
    _rpmFilters(rpmFilters)
{
    _motors[M0].init(pins.m0);
    _motors[M1].init(pins.m1);
    _motors[M2].init(pins.m2);
    _motors[M3].init(pins.m3);
    // There are a maximum of 12 rpmFilter iterations: 4 motors and up to 3 harmonics for each motor.
    // We want to complete all 12 iterations in less than 1000 microseconds.
    if (taskIntervalMicroseconds >= 1000) {
        _rpmFilterIterationCount =  12;
    } else if (taskIntervalMicroseconds >= 500) {
        _rpmFilterIterationCount =  6;
    } else if (taskIntervalMicroseconds >= 250) {
        _rpmFilterIterationCount =  3;
    } else {
        _rpmFilterIterationCount =  2;
    }
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

const DynamicIdleController* MotorMixerQuadX_DShot::getDynamicIdleController() const
{
    return &_dynamicIdleController;
}

void MotorMixerQuadX_DShot::setDynamicIdlerControllerConfig(const DynamicIdleController::config_t& config)
{
    _dynamicIdleController.setConfig(config);
}

void MotorMixerQuadX_DShot::outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount)
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

    // Output to the motors, reading the motor RPM
    // Motor outputs are converted to DShot range [47,2047]
    _motors[M0].write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M0], _motorOutputMin, 1.0F)) + 47)),
    _motors[M0].read();
    _rpmFilters.setFrequencyHzIterationStart(M0, _motors[M0].getMotorHz());

    _motors[M1].write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M1], _motorOutputMin, 1.0F)) + 47)),
    _motors[M1].read();
    _rpmFilters.setFrequencyHzIterationStart(M1, _motors[M1].getMotorHz());

    _motors[M2].write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M2], _motorOutputMin, 1.0F)) + 47)),
    _motors[M2].read();
    _rpmFilters.setFrequencyHzIterationStart(M2, _motors[M2].getMotorHz());

    _motors[M3].write(static_cast<uint16_t>(std::lroundf(2000.0F*clip(_motorOutputs[M3], _motorOutputMin, 1.0F)) + 47)),
    _motors[M3].read();
    _rpmFilters.setFrequencyHzIterationStart(M3, _motors[M3].getMotorHz());
}

void MotorMixerQuadX_DShot::rpmFilterSetFrequencyHzIterationStep()
{
    // Perform an rpmFilter iteration step for each motor
    // Note that _rpmFilters.setFrequencyHzIterationStep is an expensive calculation and runs off a state machine, setting one motor harmonic per iteration
    // so we want to call it even if we do not write to the motors
    for (size_t ii = 0; ii < _rpmFilterIterationCount; ++ii) {
        _rpmFilters.setFrequencyHzIterationStep();
    }
}
