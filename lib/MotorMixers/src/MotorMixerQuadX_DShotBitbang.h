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
    MotorMixerQuadX_DShotBitbang(uint32_t taskIntervalMicroseconds, Debug& debug, const stm32_motor_pins_t& pins, RPM_Filters& rpmFilters);
public:
    virtual void outputToMotors(commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual void rpmFilterIterationStep() override;
    virtual const DynamicIdleController* getDynamicIdleController() const override;
    virtual void setDynamicIdlerControllerConfig(const DynamicIdleController::config_t& config) override;
    float calculateSlowestMotorHz() const;
protected:
    enum { DEFAULT_MOTOR_POLE_COUNT = 14 };
    uint16_t _motorPoleCount {DEFAULT_MOTOR_POLE_COUNT}; //!< number of poles the motor has, used to calculate RPM from telemetry data
    float _eRPMtoHz {};
    DynamicIdleController _dynamicIdleController;
    RPM_Filters& _rpmFilters;

    ESC_DShotBitbang _escDShot {};
    std::array<float, MOTOR_COUNT> _motorFrequenciesHz {};
};
