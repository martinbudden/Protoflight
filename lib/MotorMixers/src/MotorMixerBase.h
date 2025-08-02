#pragma once

#include <cstddef>
#include <cstdint>

class MotorMixerBase {
public:
    struct commands_t {
        float speed;
        float roll;
        float pitch;
        float yaw;
    };
public:
    explicit MotorMixerBase(uint32_t motorCount) : _motorCount(motorCount) {}
    inline size_t getMotorCount() const { return _motorCount; }
    inline bool motorsIsOn() const { return _motorsIsOn; }
    inline void motorsSwitchOn() { _motorsIsOn = true; }
    inline void motorsSwitchOff() { _motorsIsOn = false; }
    inline bool motorsIsDisabled() const { return _motorsIsDisabled; }

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
    int32_t _motorsIsOn {false};
    int32_t _motorsIsDisabled {false};
    float _minMotorOutput {}; // minimum motor output, typically set to 5.5% to avoid ESC desynchronization 
};
