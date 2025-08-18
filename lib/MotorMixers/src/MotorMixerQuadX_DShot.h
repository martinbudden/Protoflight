#pragma once

#include <ESC_DShot.h>
#include <MotorMixerQuadX_Base.h>
#include <xyz_type.h>

class RPM_Filters;

/*!
DShot Motor Mixer.

Hz is used for motor revolutions per second rather than RPS, since RPS is generally used for Radians Per Second.
*/
class MotorMixerQuadX_DShot : public MotorMixerQuadX_Base {
public:
    MotorMixerQuadX_DShot(Debug& debug, const pins_t& pins, RPM_Filters& rpmFilters, DynamicIdleController& dynamicIdleController);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual DynamicIdleController* getDynamicIdleController() const override;
    float calculateSlowestMotorHz() const;
protected:
    RPM_Filters& _rpmFilters;
    DynamicIdleController& _dynamicIdleController;

    ESC_DShot _motorBR {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorFR {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorBL {ESC_DShot::ESC_PROTOCOL_DSHOT300};
    ESC_DShot _motorFL {ESC_DShot::ESC_PROTOCOL_DSHOT300};
};
