#include "Mixers.h"


/*!
Calculate the "mix" for the QuadX motor configuration.
Motor rotation is "propellers out".
Assumes motors are on.
Motor numbering is:
4 2
3 1
*/
float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands)
{
    enum { MOTOR_COUNT = 4 };

    const float throttle = commands.throttle;
    motorOutputs[0] = throttle - commands.roll + commands.pitch + commands.yaw; // back right
    motorOutputs[1] = throttle - commands.roll - commands.pitch - commands.yaw; // front right
    motorOutputs[2] = throttle + commands.roll + commands.pitch - commands.yaw; // back left
    motorOutputs[3] = throttle + commands.roll - commands.pitch + commands.yaw; // front left

    float maxOutput = motorOutputs[0];
    float minOutput = motorOutputs[0];
    float output = motorOutputs[1];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[2];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[3];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }

    // constrain throttle so it won't clip any outputs
    const float throttleClipped = MotorMixerBase::clip(throttle, -minOutput, 1.0F - maxOutput);
    // adjust motor outputs so none are clipped
    for (size_t ii = 0; ii < MOTOR_COUNT; ++ii) {
        motorOutputs[ii] -= (throttle - throttleClipped);
    }

    return throttleClipped;
}

/*!
Calculate the "mix" for the HexX motor configuration.
Motor rotation is "propellers out".
Assumes motors are on.
*/
float mixHexX(std::array<float, 6>& motorOutputs, const MotorMixerBase::commands_t& commands)
{
    enum { MOTOR_COUNT = 6 };

    const float throttle = commands.throttle;
    static constexpr float sin30 = 0.5F;
    static constexpr float sin60 = 0.86602540378F;
    motorOutputs[0] = throttle - sin30*commands.roll + sin60*commands.pitch + commands.yaw; // back right
    motorOutputs[1] = throttle - sin30*commands.roll - sin60*commands.pitch - commands.yaw; // front right
    motorOutputs[2] = throttle + sin30*commands.roll + sin60*commands.pitch - commands.yaw; // back left
    motorOutputs[3] = throttle + sin30*commands.roll - sin60*commands.pitch + commands.yaw; // front left
    motorOutputs[4] = throttle -       commands.roll                        + commands.yaw; // center right
    motorOutputs[5] = throttle +       commands.roll                        - commands.yaw; // center left

    float maxOutput = motorOutputs[0];
    float minOutput = motorOutputs[0];
    float output = motorOutputs[1];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[2];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[3];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[4];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[5];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }

    // constrain throttle so it won't clip any outputs
    const float throttleClipped = MotorMixerBase::clip(throttle, -minOutput, 1.0F - maxOutput);
    // adjust motor outputs so none are clipped
    for (size_t ii = 0; ii < MOTOR_COUNT; ++ii) {
        motorOutputs[ii] -= (throttle - throttleClipped);
    }

    return throttleClipped;
}

/*!
Calculate the "mix" for the OctoX motor configuration.
Motor rotation is "propellers out".
Assumes motors are on.
*/
float mixOctoX(std::array<float, 8>& motorOutputs, const MotorMixerBase::commands_t& commands)
{
    enum { MOTOR_COUNT = 8 };

    const float throttle = commands.throttle;

    motorOutputs[0] = throttle - commands.roll + commands.pitch + commands.yaw; // back right
    motorOutputs[1] = throttle - commands.roll - commands.pitch - commands.yaw; // front right
    motorOutputs[2] = throttle + commands.roll + commands.pitch - commands.yaw; // back left
    motorOutputs[3] = throttle + commands.roll - commands.pitch + commands.yaw; // front left

    motorOutputs[4] = throttle - commands.roll + commands.pitch - commands.yaw; // under back right
    motorOutputs[5] = throttle - commands.roll - commands.pitch + commands.yaw; // under front right
    motorOutputs[6] = throttle + commands.roll + commands.pitch + commands.yaw; // under back left
    motorOutputs[7] = throttle + commands.roll - commands.pitch - commands.yaw; // under front left

    float maxOutput = motorOutputs[0];
    float minOutput = motorOutputs[0];
    float output = motorOutputs[1];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[2];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[3];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[4];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[5];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[6];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }
    output = motorOutputs[7];
    if (output > maxOutput) { maxOutput = output; } else if (output < minOutput) { minOutput = output; }

    // constrain throttle so it won't clip any outputs
    const float throttleClipped = MotorMixerBase::clip(throttle, -minOutput, 1.0F - maxOutput);
    // adjust motor outputs so none are clipped
    for (size_t ii = 0; ii < MOTOR_COUNT; ++ii) {
        motorOutputs[ii] -= (throttle - throttleClipped);
    }

    return throttleClipped;
}
