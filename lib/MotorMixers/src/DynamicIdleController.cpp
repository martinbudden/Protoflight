#include "DynamicIdleController.h"
#include <Debug.h>


DynamicIdleController::DynamicIdleController(uint32_t taskIntervalMicroseconds, Debug& debug) :
    _taskIntervalMicroseconds(taskIntervalMicroseconds),
    _debug(debug)
{
}

void DynamicIdleController::setConfig(const config_t& config)
{
    _config = config;

    _maxIncrease = static_cast<float>(config.dyn_idle_max_increase) * 0.001F;

    _minimumAllowedMotorHz = static_cast<float>(config.dyn_idle_min_rpm_100) * 100.0F / 60.0F;
    _PID.setSetpoint(_minimumAllowedMotorHz);

    // use Betaflight multipliers for compatibility with Betaflight Configurator
    _PID.setP(static_cast<float>(config.dyn_idle_p_gain) * 0.00015F);

    const float deltaT = static_cast<float>(_taskIntervalMicroseconds) * 0.000001F;

    _PID.setI(static_cast<float>(config.dyn_idle_i_gain) * 0.01F * deltaT);
    // limit I-term to range [0, _maxIncrease]
    _PID.setIntegralMax(_maxIncrease);
    _PID.setIntegralMin(0.0F);

    _PID.setD(static_cast<float>(config.dyn_idle_i_gain) * 0.0000003F / deltaT);
    _DTermFilter.init(800.0F * deltaT / 20.0F); //approx 20ms D delay, arbitrarily suits many motors
}

void DynamicIdleController::setMinimumAllowedMotorHz(float minimumAllowedMotorHz)
{
    _minimumAllowedMotorHz = minimumAllowedMotorHz;
    _PID.setSetpoint(_minimumAllowedMotorHz);
}

float DynamicIdleController::calculateSpeedIncrease(float slowestMotorHz, float deltaT)
{
    if (_minimumAllowedMotorHz == 0.0F) {
        // if motors are allowed to stop, then no speed increase is needed
        return  0.0F;
    }

    const float slowestMotorHzDeltaFiltered = _DTermFilter.filter(slowestMotorHz - _PID.getPreviousMeasurement());
    float speedIncrease = _PID.updateDelta(slowestMotorHz, slowestMotorHzDeltaFiltered, deltaT);

    speedIncrease = clip(speedIncrease, 0.0F, _maxIncrease);

    if (_debug.getMode() == DEBUG_DYN_IDLE) {
        const PIDF::error_t error = _PID.getError();
        _debug.set(0, static_cast<int16_t>(std::max(-1000L, std::lroundf(error.P * 10000))));
        _debug.set(1, static_cast<int16_t>(std::lroundf(error.I * 10000)));
        _debug.set(2, static_cast<int16_t>(std::lroundf(error.D * 10000)));
    }

    return speedIncrease;
}

void DynamicIdleController::resetPID()
{
    _PID.resetIntegral();
}
