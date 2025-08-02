#pragma once

#include <MotorMixerBase.h>
#include <array>


class MotorMixerQuadX_Base : public MotorMixerBase {
public:
    enum { MOTOR_BR=0, MOTOR_FR=1, MOTOR_BL=2, MOTOR_FL=3, MOTOR_COUNT=4, MOTOR_BEGIN=0 };
    MotorMixerQuadX_Base() : MotorMixerBase(MOTOR_COUNT) {}
public:
    // 4 2
    // 3 1
    struct pins_t {
        uint8_t br;
        uint8_t fr;
        uint8_t bl;
        uint8_t fl;
    };
protected:
    std::array<float, MOTOR_COUNT> _motorOutputs {};
};
