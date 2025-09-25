#pragma once

#include <MotorMixerBase.h>
#include <array>


class MotorMixerHexBase : public MotorMixerBase {
public:
    enum { M0=0, M1=1, M2=2, M3=3, M4=4, M5=5, MOTOR_COUNT=6, MOTOR_BEGIN=0 };
    explicit MotorMixerHexBase(Debug& debug) : MotorMixerBase(MOTOR_COUNT, debug) {}
    virtual float getMotorOutput(size_t motorIndex) const override { return _motorOutputs[motorIndex]; } // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
public:
    struct motor_pins_t {
        uint8_t m0;
        uint8_t m1;
        uint8_t m2;
        uint8_t m3;
        uint8_t m4;
        uint8_t m5;
    };
    struct stm32_motor_pins_t {
        stm32_motor_pin_t m0;
        stm32_motor_pin_t m1;
        stm32_motor_pin_t m2;
        stm32_motor_pin_t m3;
        stm32_motor_pin_t m4;
        stm32_motor_pin_t m5;
    };
protected:
    std::array<float, MOTOR_COUNT> _motorOutputs {};
};
