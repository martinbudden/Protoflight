#pragma once

#include <ESC_DShot.h>
#include <Filters.h>
#include <MotorMixerQuadX_Base.h>
#include <PIDF.h>

class RPM_Filters;

/*!
DShot Motor Mixer.

Hz is used for motor revolutions per second rather than RPS, since RPS is generally used for Radians Per Second.
*/
class MotorMixerQuadX_DShot : public MotorMixerQuadX_Base {
public:
    MotorMixerQuadX_DShot (Debug& debug, const pins_t& pins, RPM_Filters& rpmFilters, uint32_t taskIntervalMicroSeconds);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual void setDynamicIdleControllerConfig(const dynamic_idle_controller_config_t& config) override;
    float calculateSlowestMotorHz() const;
protected:
    RPM_Filters& _rpmFilters;
    uint32_t _taskIntervalMicroSeconds;
    float _dynamicIdleMinimumAllowedMotorHz {}; // minimum motor Hz, dynamically controlled
    float _dynamicIdleMaxIncrease {};
    //float _dynamicIdleMaxIncreaseDelayK {};
    PIDF _dynamicIdlePID {}; // PID to dynamic idle, ie to ensure slowest motor does not go below min RPS
    PowerTransferFilter1 _dynamicIdleDTermFilter {};

    ESC_DShot _motorBR {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorFR {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorBL {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorFL {ESC_DShot::ESC_PROTOCOL_DSHOT300};
};

