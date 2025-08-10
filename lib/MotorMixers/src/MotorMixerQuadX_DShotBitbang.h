#pragma once

#include <ESC_DShotBitbang.h>
#include <MotorMixerQuadX_Base.h>

class RPM_Filters;

/*!
DShot Motor Mixer.

Hz is used for motor revolutions per second rather than RPS, since RPS is generally used for Radians Per Second.
*/
class MotorMixerQuadX_DShotBitbang : public MotorMixerQuadX_Base {
public:
    MotorMixerQuadX_DShotBitbang(Debug& debug, const pins_t& pins, RPM_Filters& rpmFilters, DynamicIdleController& dynamicIdleController);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual DynamicIdleController* getDynamicIdleController() const override;
    float calculateSlowestMotorHz() const;
protected:
    RPM_Filters& _rpmFilters;
    DynamicIdleController& _dynamicIdleController;

    ESC_DShotBitbang _escBitbang {};
};
