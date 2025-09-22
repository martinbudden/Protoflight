#pragma once

#include "MotorMixerBase.h"
#include <array>

float mixQuadX(std::array<float, 4>& motorOutputs, const MotorMixerBase::commands_t& commands, float throttleIncrease);
float mixHexX (std::array<float, 6>& motorOutputs, const MotorMixerBase::commands_t& commands, float throttleIncrease);
float mixOctoX(std::array<float, 8>& motorOutputs, const MotorMixerBase::commands_t& commands, float throttleIncrease);
