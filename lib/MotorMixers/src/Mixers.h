#pragma once

#include "MotorMixerBase.h"
#include <array>

float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin);
// variant for test code
float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin, float& undershoot, float& overshoot); // NOLINT(readability-redundant-declaration)
float mixHexX (std::array<float, 6>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin);
float mixOctoX(std::array<float, 8>& motorOutputs, const MotorMixerBase::commands_t& commands, float motorOutputMin);
