#pragma once

#include <ESC_DShot.h>
#include <MotorMixerQuadX_Base.h>
//#include <PIDF.h>


class MotorMixerQuadX_DShot : public MotorMixerQuadX_Base {
public:
    explicit MotorMixerQuadX_DShot (const pins_t& pins);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
protected:
    float _minMotorRevolutionsPerSecond {}; // minimum motor RPS, dynamically controlled
    //PIDF _motorRevolutionsPerSecondPID {}; // PID to control minimum motor RPS
    ESC_DShot _motorBR {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorFR {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorBL {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorFL {ESC_DShot::ESC_PROTOCOL_DSHOT300};
};

