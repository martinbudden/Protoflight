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
    MotorMixerQuadX_DShot(uint32_t taskIntervalMicroseconds, Debug& debug, const motor_pins_t& pins, RPM_Filters& rpmFilters);
public:
    virtual void outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual void rpmFilterSetFrequencyHzInterationStep() override;
    virtual const DynamicIdleController* getDynamicIdleController() const override;
    virtual void setDynamicIdlerControllerConfig(const DynamicIdleController::config_t& config) override;
    float calculateSlowestMotorHz() const;
protected:
    DynamicIdleController _dynamicIdleController;
    RPM_Filters& _rpmFilters;
    size_t _rpmFilterIterationCount {};
    std::array<ESC_DShot, MOTOR_COUNT> _motors;
};
