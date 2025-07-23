#pragma once

#include <Filters.h>
#include <MotorMixerQuadX_Base.h>
#include <array>


class MotorMixerQuadX_PWM : public MotorMixerQuadX_Base {
public:
    MotorMixerQuadX_PWM (const pins_t& pins, float deltaT);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
protected:
    pins_t _pins {};
    std::array<IIR_filter, MOTOR_COUNT> _motorFilters;
};
