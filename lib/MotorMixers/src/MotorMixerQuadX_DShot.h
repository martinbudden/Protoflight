#pragma once

#include <ESC_DShot.h>
#include <MotorMixerQuadBase.h>
#include <xyz_type.h> // needed or test code won't build

class RPM_Filters;

/*!
DShot Motor Mixer.

Hz is used for motor revolutions per second rather than RPS, since RPS is generally used for Radians Per Second.
*/
class MotorMixerQuadX_DShot : public MotorMixerQuadBase {
public:
    MotorMixerQuadX_DShot(Debug& debug, const motor_pins_t& pins, RPM_Filters& rpmFilters, DynamicIdleController& dynamicIdleController);
public:
    virtual void outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual DynamicIdleController* getDynamicIdleController() const override;
    float calculateSlowestMotorHz() const;
protected:
    RPM_Filters& _rpmFilters;
    DynamicIdleController& _dynamicIdleController;
    std::array<ESC_DShot, MOTOR_COUNT> _motors;
};
