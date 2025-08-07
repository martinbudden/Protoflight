#include "MotorMixerQuadX_DShot.h"
#include<Debug.h>
#include <Filters.h>
#include <RPM_Filters.h>
#include <algorithm>
#include <cmath>


MotorMixerQuadX_DShot::MotorMixerQuadX_DShot(Debug& debug, const pins_t& pins, RPM_Filters& rpmFilters, uint32_t taskIntervalMicroSeconds) :
    MotorMixerQuadX_Base(debug),
    _rpmFilters(rpmFilters),
    _taskIntervalMicroSeconds(taskIntervalMicroSeconds)
{
    _motorBR.init(pins.br);
    _motorFR.init(pins.fr);
    _motorBL.init(pins.bl);
    _motorFL.init(pins.fl);
}

void MotorMixerQuadX_DShot::setDynamicIdleControllerConfig(const dynamic_idle_controller_config_t& config)
{
    _dynamicIdleControllerConfig = config;

    _dynamicIdleMinimumAllowedMotorHz = static_cast<float>(config.minRPM) * 100.0F / 60.0F;
    _dynamicIdleMaxIncrease = static_cast<float>(config.maxIncrease) * 0.001F;

    _dynamicIdlePID.setSetpoint(_dynamicIdleMinimumAllowedMotorHz);
    // use Betaflight multipliers for compatibility with Betaflight Configurator
    _dynamicIdlePID.setP(static_cast<float>(config.kp) * 0.00015F);
    const float deltaT = static_cast<float>(_taskIntervalMicroSeconds) * 0.000001F;
    _dynamicIdlePID.setI(static_cast<float>(config.ki) * 0.01F * deltaT);
    // limit I-term to range [0, _dynamicIdleMaxIncrease]
    _dynamicIdlePID.setIntegralMax(_dynamicIdleMaxIncrease);
    _dynamicIdlePID.setIntegralMin(0.0F);

#if false
    _dynamicIdlePID.setD(static_cast<float>(config.kd) * 0.0000003F / deltaT);
    _minHzDelayK = 800 * deltaT / 20.0F; //approx 20ms D delay, arbitrarily suits many motors
#else
    //_dynamicIdlePID.setD(static_cast<float>(config.kd) * (0.0000003F / deltaT) * (800 * deltaT / 20.0F);
    _dynamicIdlePID.setD(static_cast<float>(config.kd) * 0.000012F / (deltaT * deltaT));
#endif
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

void MotorMixerQuadX_DShot::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)tickCount;

    float speedIncrease = 0.0F;

    if (_dynamicIdleMinimumAllowedMotorHz > 0.0F) {
        /*
        Dynamic Idle: use PID controller to boost motor speeds so that slowest motor does not go below minimum allowed RPM
 
        A minimum RPM is required because the ESC will desynchronize if the motors turn too slowly (since they won't generate
        enough back EMF for the ESC know the position of the rotor relative to the windings).

        Note that a simple minimum output value is not sufficient: consider the case where the throttle is cut while hovering,
        the quad will start to fall and this falling will generate a reverse torque on the motors which will eventually
        overcome the fixed output value. Many types of maneuver can generate this reverse torque.

        Instead we have a PID controller that increases output to the motors as the slowest motor nears the minimum allowed RPM.
        */

        const float slowestMotorHz = calculateSlowestMotorHz();
        const float slowestMotorHzDeltaFiltered = _dynamicIdleDTermFilter.filter(slowestMotorHz - _dynamicIdlePID.getPreviousMeasurement());
        //slowestMotorHzDelta = _dynamicIdledelayK * _dynamicIdleDTermFilter.filter(slowestMotorHzDelta);
        speedIncrease = _dynamicIdlePID.updateDelta(slowestMotorHz, slowestMotorHzDeltaFiltered, deltaT);

        //!!TODO: need to check this is scaled to the correct units
        speedIncrease = clip(speedIncrease, 0.0F, _dynamicIdleMaxIncrease);
        if (_debug.getMode() == DEBUG_DYN_IDLE) {
            const PIDF::error_t error = _dynamicIdlePID.getError();
            _debug.set(0, static_cast<int16_t>(std::max(-1000L, std::lroundf(error.P * 10000))));
            _debug.set(1, static_cast<int16_t>(std::lroundf(error.I * 10000)));
            _debug.set(2, static_cast<int16_t>(std::lroundf(error.D * 10000)));
        }
    }

    if (motorsIsOn()) {
        // calculate the "mix" for the QuadX motor configuration
        const float speed = commands.speed + speedIncrease;
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
