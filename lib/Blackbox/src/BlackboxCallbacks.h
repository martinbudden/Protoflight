#pragma once

#include <BlackboxCallbacksBase.h>

class AHRS_MessageQueue;
class Cockpit;
class Debug;
class FlightController;
class RC_Modes;
class ReceiverBase;


class BlackboxCallbacks : public BlackboxCallbacksBase {
public:
    BlackboxCallbacks(const AHRS_MessageQueue& messageQueue, const FlightController& flightController, const Cockpit& cockpit, const ReceiverBase& receiver, const Debug& debug);
public:
    virtual void loadSlowState(blackbox_slow_state_t& blackboxSlowState) override;
    virtual void loadMainState(blackbox_main_state_t& blackboxMainState, uint32_t currentTimeUs) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxModeActive() const override;
    virtual bool isBlackboxEraseModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroseconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;
private:
    const AHRS_MessageQueue& _messageQueue;
    const FlightController& _flightController;
    const Cockpit& _cockpit;
    const RC_Modes& _rcModes;
    const ReceiverBase& _receiver;
    const Debug& _debug;
};
