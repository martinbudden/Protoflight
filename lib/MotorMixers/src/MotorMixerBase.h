#pragma once

#include <cstddef>
#include <cstdint>

class Debug;
class DynamicIdleController;

class MotorMixerBase {
public:
    struct commands_t {
        float speed;
        float roll;
        float pitch;
        float yaw;
    };
public:
    MotorMixerBase(uint32_t motorCount, Debug& debug) : _motorCount(motorCount), _debug(debug) {}
    inline size_t getMotorCount() const { return _motorCount; }
    inline bool motorsIsOn() const { return _motorsIsOn; }
    inline void motorsSwitchOn() { _motorsIsOn = true; }
    inline void motorsSwitchOff() { _motorsIsOn = false; }
    inline bool motorsIsDisabled() const { return _motorsIsDisabled; }

    void setMotorOutputMin(float motorOutputMin) { _motorOutputMin = motorOutputMin; }
    float getMotorOutputMin() const { return _motorOutputMin; }

    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) { (void)commands; (void)deltaT; (void)tickCount; }
    virtual float getMotorOutput(size_t motorIndex) const { (void)motorIndex; return 0.0F; }

    virtual int32_t getMotorRPM(size_t motorIndex) const { (void)motorIndex; return 0; }
    virtual float getMotorFrequencyHz(size_t motorIndex) const { (void)motorIndex; return 0; }

    virtual DynamicIdleController* getDynamicIdleController() const { return nullptr; }
public:
    static inline float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
protected:
    const size_t _motorCount;
    Debug& _debug;
    int32_t _motorsIsOn {false};
    int32_t _motorsIsDisabled {false};
    float _motorOutputMin {0.0F}; // minimum motor output, typically set to 5.5% to avoid ESC desynchronization
};
