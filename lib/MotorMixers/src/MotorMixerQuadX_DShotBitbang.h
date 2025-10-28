#pragma once

#include "ESC_DShotBitbang.h"
#include "MotorMixerQuadBase.h"
#include "RPM_Filters.h"


/*!
DShot Motor Mixer.

Hz is used for motor revolutions per second rather than RPS, since RPS is generally used for Radians Per Second.
*/
class MotorMixerQuadX_DShotBitbang : public MotorMixerQuadBase {
public:
    MotorMixerQuadX_DShotBitbang(uint32_t taskIntervalMicroseconds, uint32_t outputToMotorsDenominator, Debug& debug, const stm32_motor_pins_t& pins);
public:
    virtual void outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual void rpmFilterSetFrequencyHzIterationStep() override;
    virtual RPM_Filters* getRPM_Filters() override;
    virtual const RPM_Filters* getRPM_Filters() const override;
    virtual const DynamicIdleController* getDynamicIdleController() const override;
    virtual void setDynamicIdlerControllerConfig(const DynamicIdleController::config_t& config) override;
    float calculateSlowestMotorHz() const;
protected:
    enum { DEFAULT_MOTOR_POLE_COUNT = 14 };
    uint16_t _motorPoleCount {DEFAULT_MOTOR_POLE_COUNT}; //!< number of poles the motor has, used to calculate RPM from telemetry data
    float _eRPMtoHz {};
    DynamicIdleController _dynamicIdleController;
    RPM_Filters _rpmFilters;
    size_t _rpmFilterIterationCount {};

    ESC_DShotBitbang _escDShot {};
    std::array<float, MOTOR_COUNT> _motorFrequenciesHz {};
};
