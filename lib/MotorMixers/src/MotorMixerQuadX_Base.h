#pragma once

#include <MotorMixerBase.h>
#include <array>


class MotorMixerQuadX_Base : public MotorMixerBase {
public:
    enum { MOTOR_BR=0, MOTOR_FR=1, MOTOR_BL=2, MOTOR_FL=3, MOTOR_COUNT=4, MOTOR_BEGIN=0 };
    explicit MotorMixerQuadX_Base(Debug& debug) : MotorMixerBase(MOTOR_COUNT, debug) {}
    virtual float getMotorOutput(size_t motorIndex) const override { return _motorOutputs[motorIndex]; } // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
public:
    struct port_pin_t {
        uint8_t port;
        uint8_t pin;
    };
    // 4 2
    // 3 1
    struct pins_t {
        uint8_t br;
        uint8_t fr;
        uint8_t bl;
        uint8_t fl;
    };
    struct port_pins_t {
        port_pin_t br;
        port_pin_t fr;
        port_pin_t bl;
        port_pin_t fl;
    };
protected:
    std::array<float, MOTOR_COUNT> _motorOutputs {};
};
