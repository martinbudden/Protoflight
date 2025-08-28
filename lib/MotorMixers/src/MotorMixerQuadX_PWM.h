#pragma once

#include <MotorMixerQuadX_Base.h>


class MotorMixerQuadX_PWM : public MotorMixerQuadX_Base {
public:
    MotorMixerQuadX_PWM(Debug& debug, const pins_t& pins);
    MotorMixerQuadX_PWM(Debug& debug, const port_pins_t& pins);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
    void writeMotorPWM(const port_pin_t& pin, uint8_t channel);
protected:
    float _pwmScale {255.0F};
    port_pins_t _pins {};
};
