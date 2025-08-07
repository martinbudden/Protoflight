#pragma once

#include <Filters.h>
#include <PIDF.h>
#include <cstdint>

class Debug;


/*!
Dynamic Idle: use PID controller to boost motor speeds so that slowest motor does not go below minimum allowed RPM

A minimum RPM is required because the ESC will desynchronize if the motors turn too slowly (since they won't generate
enough back EMF for the ESC know the position of the rotor relative to the windings).

Note that a simple minimum output value is not sufficient: consider the case where the throttle is cut while hovering,
the quad will start to fall and this falling will generate a reverse torque on the motors which will eventually
overcome the fixed output value. Many types of maneuver can generate this reverse torque.

Instead we have a PID controller that increases output to the motors as the slowest motor nears the minimum allowed RPM.
*/
class DynamicIdleController {
public:
    struct config_t {
        uint8_t dyn_idle_min_rpm_100; // multiply this by 100 to get the actual min RPM
        uint8_t dyn_idle_p_gain;
        uint8_t dyn_idle_i_gain;
        uint8_t dyn_idle_d_gain;
        uint8_t dyn_idle_max_increase;
    };
public:
    DynamicIdleController(const config_t& config, uint32_t taskIntervalMicroSeconds, Debug& debug);
    void setConfig(const config_t& config, uint32_t taskIntervalMicroseconds);
    const config_t& getConfig() const { return _config; }
    void setMinimumAllowedMotorHz(float minimumAllowedMotorHz);
    float getMinimumAllowedMotorHz() { return _minimumAllowedMotorHz; }
    float calculateSpeedIncrease(float slowestMotorHz, float deltaT);
    void resetPID(); //!< for test code
public:
    static inline float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
private:
    float _minimumAllowedMotorHz {}; // minimum motor Hz, dynamically controlled
    float _maxIncrease {};
    //float _dynamicIdleMaxIncreaseDelayK {};
    PIDF _PID {}; // PID to dynamic idle, ie to ensure slowest motor does not go below min RPS
    PowerTransferFilter1 _DTermFilter {};
    Debug& _debug;
    config_t _config {};
};
