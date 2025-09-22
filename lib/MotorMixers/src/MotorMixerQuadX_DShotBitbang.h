#pragma once

#include <ESC_DShotBitbang.h>
#include <MotorMixerQuadBase.h>

class RPM_Filters;

/*!
DShot Motor Mixer.

Hz is used for motor revolutions per second rather than RPS, since RPS is generally used for Radians Per Second.
*/
class MotorMixerQuadX_DShotBitbang : public MotorMixerQuadBase {
public:
    MotorMixerQuadX_DShotBitbang(Debug& debug, const pins_t& pins, RPM_Filters& rpmFilters, DynamicIdleController& dynamicIdleController);
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual DynamicIdleController* getDynamicIdleController() const override;
    float calculateSlowestMotorHz() const;
protected:
    enum { DEFAULT_MOTOR_POLE_COUNT = 14 };
    uint16_t _motorPoleCount {DEFAULT_MOTOR_POLE_COUNT}; //!< number of poles the motor has, used to calculate RPM from telemetry data
    float _eRPMtoHz {};
    RPM_Filters& _rpmFilters;
    DynamicIdleController& _dynamicIdleController;

    ESC_DShotBitbang _escDShot {};
    std::array<float, MOTOR_COUNT> _motorFrequenciesHz {};
};
