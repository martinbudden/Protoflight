#pragma once

#include <MotorMixerQuadX_Base.h>


class MotorMixerQuadX_PWM : public MotorMixerQuadX_Base {
public:
    MotorMixerQuadX_PWM (Debug& debug, const pins_t& pins);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
protected:
    pins_t _pins {};
};
