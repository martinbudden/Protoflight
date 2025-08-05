#include "MotorMixerQuadX_DShot.h"
#include<Debug.h>
#include <Filters.h>
#include <RPM_Filter.h>
#include <algorithm>
#include <cmath>


MotorMixerQuadX_DShot::MotorMixerQuadX_DShot(Debug& debug, const pins_t& pins, RPM_Filter& rpmFilter, uint32_t taskIntervalMicroSeconds) :
    MotorMixerQuadX_Base(debug),
    _rpmFilter(rpmFilter),
    _taskIntervalMicroSeconds(taskIntervalMicroSeconds)
{
    _motorBR.init(pins.br);
    _motorFR.init(pins.fr);
    _motorBL.init(pins.bl);
    _motorFL.init(pins.fl);
}

void MotorMixerQuadX_DShot::setDynamicIdleControllerConfig(const dynamic_idle_controller_config_t& config)
{
    _minRPS = config.minRPM * 100.0f / 60.0f;
    _minRPS_MaxIncrease = config.maxIncrease * 0.001f;

    _minRPS_PID.setSetpoint(_minRPS);
    _minRPS_PID.setP(config.kp * 0.00015f);
    const float deltaT = static_cast<float>(_taskIntervalMicroSeconds) * 0.000001F;
    _minRPS_PID.setI(config.ki * 0.01f * deltaT);
    _minRPS_PID.setD(config.kd * 0.0000003f / deltaT);
}

float MotorMixerQuadX_DShot::calculateMinMotorRPS() const
{
    float minRPS = _motorBR.getMotorHz();
    float motorRPS = _motorFR.getMotorHz();
    if (motorRPS < minRPS) {
        minRPS = motorRPS;
    }
    motorRPS = _motorBL.getMotorHz();
    if (motorRPS < minRPS) {
        minRPS = motorRPS;
    }
    motorRPS = _motorFL.getMotorHz();
    if (motorRPS < minRPS) {
        minRPS = motorRPS;
    }
    return minRPS;
}

void MotorMixerQuadX_DShot::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)deltaT;
    (void)tickCount;
    float motorOutputMin = _motorOutputMin;
    if (_minRPS > 0.0F) {
        float minRPS = calculateMinMotorRPS();
        float minRPS_Delta = minRPS - _minRPS_PID.getPreviousMeasurement();
        minRPS_Delta = _minRPS_DTermFilter.filter(minRPS_Delta);
        float motorRPS_Increase = _minRPS_PID.updateDelta(minRPS, minRPS_Delta, deltaT);

        motorRPS_Increase = clip(motorRPS_Increase, 0.0F, _minRPS_MaxIncrease);
        motorOutputMin = motorRPS_Increase * 1000.0F;
        if (_debug.getMode() == DEBUG_DYN_IDLE) {
            const PIDF::error_t error = _minRPS_PID.getError();
            _debug.set(0, static_cast<int16_t>(std::max(-1000L, std::lroundf(error.P * 10000))));
            _debug.set(1, static_cast<int16_t>(std::lroundf(error.I * 10000)));
            _debug.set(2, static_cast<int16_t>(std::lroundf(error.D * 10000)));
        }
    }

    if (motorsIsOn()) {
        // calculate the "mix" for the QuadX motor configuration
        _motorOutputs[MOTOR_BR] = -commands.roll - commands.pitch - commands.yaw + commands.speed;
        _motorOutputs[MOTOR_FR] = -commands.roll + commands.pitch + commands.yaw + commands.speed;
        _motorOutputs[MOTOR_BL] =  commands.roll - commands.pitch + commands.yaw + commands.speed;
        _motorOutputs[MOTOR_FL] =  commands.roll + commands.pitch - commands.yaw + commands.speed;

        // scale motor output to [0.0F, 1000.0F], which is the range required for DShot
        _motorOutputs[MOTOR_BR] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_BR], motorOutputMin, 1.0F));
        _motorOutputs[MOTOR_FR] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_FR], motorOutputMin, 1.0F));
        _motorOutputs[MOTOR_BL] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_BL], motorOutputMin, 1.0F));
        _motorOutputs[MOTOR_FL] =  roundf(1000.0F*clip(_motorOutputs[MOTOR_FL], motorOutputMin, 1.0F));

    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }
    // and finally output to the motors and read to get the motor RPM from bidirectional dshot
    _motorBR.write(static_cast<uint16_t>(_motorOutputs[MOTOR_BR]));
    _motorBR.read();
    _rpmFilter.setFrequency(MOTOR_BR, _motorBR.getMotorHz());

    _motorFR.write(static_cast<uint16_t>(_motorOutputs[MOTOR_FR]));
    _motorFR.read();
    _rpmFilter.setFrequency(MOTOR_FR, _motorBR.getMotorHz());

    _motorBL.write(static_cast<uint16_t>(_motorOutputs[MOTOR_BL]));
    _motorBL.read();
    _rpmFilter.setFrequency(MOTOR_BL, _motorBR.getMotorHz());

    _motorFL.write(static_cast<uint16_t>(_motorOutputs[MOTOR_FL]));
    _motorFL.read();
    _rpmFilter.setFrequency(MOTOR_FL, _motorBR.getMotorHz());
}
