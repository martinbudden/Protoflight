#include "MotorMixerHexX_Base.h"


void MotorMixerHexX_Base::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)deltaT;
    (void)tickCount;

    _throttleCommand = commands.throttle;

    if (motorsIsOn()) {
        // calculate the "mix" for the HexX motor configuration
        const float throttle = commands.throttle;
        _motorOutputs[M0] = throttle - 0.5F*commands.roll + 0.866025F*commands.pitch + commands.yaw; // back right
        _motorOutputs[M1] = throttle - 0.5F*commands.roll - 0.866025F*commands.pitch - commands.yaw; // front right
        _motorOutputs[M2] = throttle + 0.5F*commands.roll + 0.866025F*commands.pitch - commands.yaw; // back left
        _motorOutputs[M3] = throttle + 0.5F*commands.roll - 0.866025F*commands.pitch + commands.yaw; // front left 
        _motorOutputs[M4] = throttle -      commands.roll                            + commands.yaw; // right
        _motorOutputs[M5] = throttle +      commands.roll                            - commands.yaw; // left 
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
    }

    writeMotor(M0);
    writeMotor(M1);
    writeMotor(M2);
    writeMotor(M3);
    writeMotor(M4);
    writeMotor(M5);
}
