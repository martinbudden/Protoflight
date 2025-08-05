#pragma once

#include <cstddef>
#include <cstdint>

class Debug;


class MotorMixerBase {
public:
    struct commands_t {
        float speed;
        float roll;
        float pitch;
        float yaw;
    };
    struct dynamic_idle_controller_config_t {
        uint8_t minRPM; // minimum motor speed enforced by the dynamic idle controller
        // PID constants
        uint8_t kp;
        uint8_t ki;
        uint8_t kd;
        uint8_t maxIncrease; // limit on maximum possible increase in motor idle drive during active control
    };

public:
    explicit MotorMixerBase(uint32_t motorCount, Debug& debug) : _motorCount(motorCount), _debug(debug) {}
    inline size_t getMotorCount() const { return _motorCount; }
    inline bool motorsIsOn() const { return _motorsIsOn; }
    inline void motorsSwitchOn() { _motorsIsOn = true; }
    inline void motorsSwitchOff() { _motorsIsOn = false; }
    inline bool motorsIsDisabled() const { return _motorsIsDisabled; }

    virtual void setDynamicIdleControllerConfig(const dynamic_idle_controller_config_t& dynamicIdleControllerConfig)
        { (void)dynamicIdleControllerConfig; }
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
        { (void)commands; (void)deltaT; (void)tickCount; }
    virtual float getMotorOutput(size_t motorIndex) const
        { (void)motorIndex; return 0.0F; }
    virtual int32_t getMotorRPM(size_t motorIndex) const
        { (void)motorIndex; return 0; }
    virtual float getMotorFrequencyHz(size_t motorIndex) const
        { (void)motorIndex; return 0; }
public:
    static float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
protected:
    const size_t _motorCount;
    Debug& _debug;
    int32_t _motorsIsOn {false};
    int32_t _motorsIsDisabled {false};
    float _minMotorOutput {}; // minimum motor output, typically set to 5.5% to avoid ESC desynchronization 
};
