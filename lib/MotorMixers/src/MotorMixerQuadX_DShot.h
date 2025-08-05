#pragma once

#include <ESC_DShot.h>
#include <Filters.h>
#include <MotorMixerQuadX_Base.h>
#include <PIDF.h>

class RPM_Filter;


class MotorMixerQuadX_DShot : public MotorMixerQuadX_Base {
public:
    MotorMixerQuadX_DShot (Debug& debug, const pins_t& pins, RPM_Filter& rpmFilter, uint32_t taskIntervalMicroSeconds);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual void setDynamicIdleControllerConfig(const dynamic_idle_controller_config_t& config) override;
    float calculateMinMotorRPS() const;
protected:
    RPM_Filter& _rpmFilter;
    uint32_t _taskIntervalMicroSeconds;
    float _motorOutputMin {0.048F}; // minimum motor speed when dynamic idling off
    float _minRPS {}; // minimum motor RPS, dynamically controlled
    float _minRPS_MaxIncrease {};
    PIDF _minRPS_PID {}; // PID to control minimum motor RPS
    PowerTransferFilter1 _minRPS_DTermFilter {};

    ESC_DShot _motorBR {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorFR {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorBL {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorFL {ESC_DShot::ESC_PROTOCOL_DSHOT300};
};

