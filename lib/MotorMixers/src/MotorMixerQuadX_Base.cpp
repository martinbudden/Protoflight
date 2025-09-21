#include "MotorMixerQuadX_PWM.h"

/*!
Calculate and output motor mix.
Note: default motor rotation is "propellers out".
*/
void MotorMixerQuadX_Base::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)deltaT;
    (void)tickCount;

    _throttleCommand = commands.throttle;

    if (motorsIsOn()) {
        // calculate the "mix" for the QuadX motor configuration
        const float throttle = commands.throttle;
        _motorOutputs[M0] = throttle - commands.roll + commands.pitch + commands.yaw; // back right
        _motorOutputs[M1] = throttle - commands.roll - commands.pitch - commands.yaw; // front right
        _motorOutputs[M2] = throttle + commands.roll + commands.pitch - commands.yaw; // back left
        _motorOutputs[M3] = throttle + commands.roll - commands.pitch + commands.yaw; // front left 
    } else {
        _motorOutputs = { 0.0F, 0.0F, 0.0F, 0.0F };
    }

    writeMotor(M0);
    writeMotor(M1);
    writeMotor(M2);
    writeMotor(M3);
}
