#include "Mixers.h"
#include <algorithm>


/*!
Calculate the "mix" for the QuadX motor configuration.
Motor rotation is "propellers out".
Assumes motors are on.
Motor numbering is:
4 2
3 1
*/
float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin)
{
    float undershoot = 0.0F;
    float overshoot = 0.0F;
    return mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
}

float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin, float& undershoot, float& overshoot) // NOLINT(readability-function-cognitive-complexity)
{
    (void)motorOutputMin;

    enum { MOTOR_COUNT = 4 };

    // calculate the motor outputs without yaw applied
    float throttle = commands.throttle;
    motorOutputs[0] = throttle - commands.roll + commands.pitch;// + commands.yaw; // back right
    motorOutputs[1] = throttle - commands.roll - commands.pitch;// - commands.yaw; // front right
    motorOutputs[2] = throttle + commands.roll + commands.pitch;// - commands.yaw; // back left
    motorOutputs[3] = throttle + commands.roll - commands.pitch;// + commands.yaw; // front left

    // deal with yaw undershoot and overshoot
    // High values of yaw can cause motor outputs to go outside range [motorOutputMin, motorOutputMax]
    // If this happens, we reduce the magnitude of the yaw command.
    // This reduces yaw authority, but avoids "yaw jumps"
    static constexpr float motorOutputMax = 1.0F;
    overshoot = 0.0F;
    undershoot = 0.0F;
    float commandYaw = commands.yaw;
    if (commands.yaw > 0.0F) {
        // check if M1 or M2 will have output less than motorOutputMin
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 < motorOutputMin) {
            undershoot = m1 - motorOutputMin;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 < motorOutputMin) {
            undershoot = std::min(undershoot, m2 - motorOutputMin);
        }
        // check if M0 or M3 will have output greater than motorOutputMax
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 > motorOutputMax) {
            overshoot = m0 - motorOutputMax;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 > motorOutputMax) {
            overshoot = std::max(overshoot, m3 - motorOutputMax);
        }
        if (commandYaw + (undershoot - overshoot) > 0.0F) {
            commandYaw += (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    } else {
        // check if M0 or M3 will have output less than motorOutputMin
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 < motorOutputMin) {
            undershoot = m0 - motorOutputMin;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 < motorOutputMin) {
            undershoot = std::min(undershoot, m3 - motorOutputMin);
        }
        // check if M1 or M2 will have output greater than motorOutputMax
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 > motorOutputMax) {
            overshoot = m1 - motorOutputMax;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 > motorOutputMax) {
            overshoot = std::max(overshoot, m2 - motorOutputMax);
        }
        if (commandYaw - (undershoot - overshoot) < 0.0F) {
            commandYaw -= (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    }

    motorOutputs[0] += commandYaw;
    motorOutputs[1] -= commandYaw;
    motorOutputs[2] -= commandYaw;
    motorOutputs[3] += commandYaw;

    float maxOutput = motorOutputs[0];
    float output = motorOutputs[1];
    if (output > maxOutput) { maxOutput = output; }
    output = motorOutputs[2];
    if (output > maxOutput) { maxOutput = output; }
    output = motorOutputs[3];
    if (output > maxOutput) { maxOutput = output; }

    if (maxOutput > motorOutputMax) {
        const float correction = maxOutput - motorOutputMax;
        throttle -= correction;
        for (auto& motorOutput : motorOutputs) {
            motorOutput -= correction; // cppcheck-suppress useStlAlgorithm
        }
    }

    return throttle;
}

/*!
Calculate the "mix" for the HexX motor configuration.
Motor rotation is "propellers out".
Assumes motors are on.
*/
float mixHexX(std::array<float, 6>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin) // NOLINT(readability-function-cognitive-complexity)
{
    (void)motorOutputMin;

    enum { MOTOR_COUNT = 6 };

    // calculate the motor outputs without yaw applied
    float throttle = commands.throttle;
    static constexpr float sin30 = 0.5F;
    static constexpr float sin60 = 0.86602540378F;
    motorOutputs[0] = throttle - sin30*commands.roll + sin60*commands.pitch;// + commands.yaw; // back right
    motorOutputs[1] = throttle - sin30*commands.roll - sin60*commands.pitch;// - commands.yaw; // front right
    motorOutputs[2] = throttle + sin30*commands.roll + sin60*commands.pitch;// - commands.yaw; // back left
    motorOutputs[3] = throttle + sin30*commands.roll - sin60*commands.pitch;// + commands.yaw; // front left
    motorOutputs[4] = throttle -       commands.roll;//                        + commands.yaw; // center right
    motorOutputs[5] = throttle +       commands.roll;//                        - commands.yaw; // center left

    // deal with yaw undershoot and overshoot
    // High values of yaw can cause motor outputs to go outside range [motorOutputMin, motorOutputMax]
    // If this happens, we reduce the magnitude of the yaw command.
    // This reduces yaw authority, but avoids "yaw jumps"
    static constexpr float motorOutputMax = 1.0F;
    float overshoot = 0.0F;
    float undershoot = 0.0F;
    float commandYaw = commands.yaw;
    if (commands.yaw > 0.0F) {
        // check if M1, M2, or M5 will have output less than motorOutputMin
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 < motorOutputMin) {
            undershoot = m1 - motorOutputMin;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 < motorOutputMin) {
            undershoot = std::min(undershoot, m2 - motorOutputMin);
        }
        const float m5 = motorOutputs[5] - commands.yaw;
        if (m5 < motorOutputMin) {
            undershoot = std::min(undershoot, m5 - motorOutputMin);
        }
        // check if M0, M3, or M4 will have output greater than motorOutputMax
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 > motorOutputMax) {
            overshoot = m0 - motorOutputMax;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 > motorOutputMax) {
            overshoot = std::max(overshoot, m3 - motorOutputMax);
        }
        const float m4 = motorOutputs[4] + commands.yaw;
        if (m4 > motorOutputMax) {
            overshoot = std::max(overshoot, m4 - motorOutputMax);
        }
        if (commandYaw + (undershoot - overshoot) > 0.0F) {
            commandYaw += (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    } else {
        // check if M0, M3, or M4 will have output less than motorOutputMin
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 < motorOutputMin) {
            undershoot = m0 - motorOutputMin;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 < motorOutputMin) {
            undershoot = std::min(undershoot, m3 - motorOutputMin);
        }
        const float m4 = motorOutputs[4] + commands.yaw;
        if (m4 < motorOutputMin) {
            undershoot = std::min(undershoot, m4 - motorOutputMin);
        }
        // check if M1, M2, or M5 will have output greater than motorOutputMax
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 > motorOutputMax) {
            overshoot = m1 - motorOutputMax;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 > motorOutputMax) {
            overshoot = std::max(overshoot, m2 - motorOutputMax);
        }
        const float m5 = motorOutputs[5] - commands.yaw;
        if (m5 > motorOutputMax) {
            overshoot = std::max(overshoot, m5 - motorOutputMax);
        }
        if (commandYaw - (undershoot - overshoot) < 0.0F) {
            commandYaw -= (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    }

    // adjust commands.yaw to remove undershoot and overshoot
    motorOutputs[0] += commandYaw;
    motorOutputs[1] -= commandYaw;
    motorOutputs[2] -= commandYaw;
    motorOutputs[3] += commandYaw;
    motorOutputs[4] += commandYaw;
    motorOutputs[5] -= commandYaw;

    const float maxOutput = *std::max_element(motorOutputs.begin(), motorOutputs.end());

    if (maxOutput > motorOutputMax) {
        const float correction = maxOutput - motorOutputMax;
        throttle -= correction;
        for (auto& motorOutput : motorOutputs) {
            motorOutput -= correction; // cppcheck-suppress useStlAlgorithm
        }
    }

    return throttle;
}

/*!
Calculate the "mix" for the OctoX motor configuration.
Motor rotation is "propellers out".
Assumes motors are on.
*/
float mixOctoX(std::array<float, 8>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin) // NOLINT(readability-function-cognitive-complexity)
{
    (void)motorOutputMin;

    enum { MOTOR_COUNT = 8 };

    float throttle = commands.throttle;

    motorOutputs[0] = throttle - commands.roll + commands.pitch;// + commands.yaw; // back right
    motorOutputs[1] = throttle - commands.roll - commands.pitch;// - commands.yaw; // front right
    motorOutputs[2] = throttle + commands.roll + commands.pitch;// - commands.yaw; // back left
    motorOutputs[3] = throttle + commands.roll - commands.pitch;// + commands.yaw; // front left

    motorOutputs[4] = throttle - commands.roll + commands.pitch;// - commands.yaw; // under back right
    motorOutputs[5] = throttle - commands.roll - commands.pitch;// + commands.yaw; // under front right
    motorOutputs[6] = throttle + commands.roll + commands.pitch;// + commands.yaw; // under back left
    motorOutputs[7] = throttle + commands.roll - commands.pitch;// - commands.yaw; // under front left

    static constexpr float motorOutputMax = 1.0F;
    float overshoot = 0.0F;
    float undershoot = 0.0F;
    float commandYaw = commands.yaw;
    if (commands.yaw > 0.0F) {
        // check if M1, M2, M4, or M7 will have output less than motorOutputMin
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 < motorOutputMin) {
            undershoot = m1 - motorOutputMin;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 < motorOutputMin) {
            undershoot = std::min(undershoot, m2 - motorOutputMin);
        }
        const float m4 = motorOutputs[4] - commands.yaw;
        if (m4 < motorOutputMin) {
            undershoot = std::min(undershoot, m4 - motorOutputMin);
        }
        const float m7 = motorOutputs[7] - commands.yaw;
        if (m7 < motorOutputMin) {
            undershoot = std::min(undershoot, m7 - motorOutputMin);
        }
        // check if M0, M3, M5, or M6 will have output greater than motorOutputMax
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 > motorOutputMax) {
            overshoot = m0 - motorOutputMax;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 > motorOutputMax) {
            overshoot = std::max(overshoot, m3 - motorOutputMax);
        }
        const float m5 = motorOutputs[5] + commands.yaw;
        if (m5 > motorOutputMax) {
            overshoot = std::max(overshoot, m5 - motorOutputMax);
        }
        const float m6 = motorOutputs[6] + commands.yaw;
        if (m6 > motorOutputMax) {
            overshoot = std::max(overshoot, m6 - motorOutputMax);
        }
        if (commandYaw + (undershoot - overshoot) > 0.0F) {
            commandYaw += (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    } else {
        // check if M0, M3, M5, or M6  will have output less than motorOutputMin
        const float m0 = motorOutputs[0] + commands.yaw;
        if (m0 < motorOutputMin) {
            undershoot = m0 - motorOutputMin;
        }
        const float m3 = motorOutputs[3] + commands.yaw;
        if (m3 < motorOutputMin) {
            undershoot = std::min(undershoot, m3 - motorOutputMin);
        }
        const float m5 = motorOutputs[5] + commands.yaw;
        if (m5 < motorOutputMin) {
            undershoot = std::min(undershoot, m5 - motorOutputMin);
        }
        const float m6 = motorOutputs[6] + commands.yaw;
        if (m6 < motorOutputMin) {
            undershoot = std::min(undershoot, m6 - motorOutputMin);
        }
        // check if M1, M2, M4, or M7 will have output greater than motorOutputMax
        const float m1 = motorOutputs[1] - commands.yaw;
        if (m1 > motorOutputMax) {
            overshoot = m1 - motorOutputMax;
        }
        const float m2 = motorOutputs[2] - commands.yaw;
        if (m2 > motorOutputMax) {
            overshoot = std::max(overshoot, m2 - motorOutputMax);
        }
        const float m4 = motorOutputs[4] - commands.yaw;
        if (m4 > motorOutputMax) {
            overshoot = std::max(overshoot, m4 - motorOutputMax);
        }
        const float m7 = motorOutputs[7] - commands.yaw;
        if (m7 > motorOutputMax) {
            overshoot = std::max(overshoot, m7 - motorOutputMax);
        }
        if (commandYaw - (undershoot - overshoot) < 0.0F) {
            commandYaw -= (undershoot - overshoot);
            throttle -= (undershoot + overshoot);
        }
    }

    motorOutputs[0] += commandYaw;
    motorOutputs[1] -= commandYaw;
    motorOutputs[2] -= commandYaw;
    motorOutputs[3] += commandYaw;
    motorOutputs[4] -= commandYaw;
    motorOutputs[5] += commandYaw;
    motorOutputs[6] += commandYaw;
    motorOutputs[7] -= commandYaw;

    const float maxOutput = *std::max_element(motorOutputs.begin(), motorOutputs.end());

    if (maxOutput > motorOutputMax) {
        const float correction = maxOutput - motorOutputMax;
        throttle -= correction;
        for (auto& motorOutput : motorOutputs) {
            motorOutput -= correction; // cppcheck-suppress useStlAlgorithm
        }
    }

    return throttle;
}
