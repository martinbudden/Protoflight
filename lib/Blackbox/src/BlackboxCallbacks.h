#pragma once

#include <BlackboxCallbacksBase.h>

class AHRS;
class BlackboxMessageQueue;
class Debug;
class FlightController;
class RadioController;
class ReceiverBase;


class BlackboxCallbacks : public BlackboxCallbacksBase {
public:
    BlackboxCallbacks(BlackboxMessageQueue& messageQueue, const AHRS& ahrs, const FlightController& flightController, const RadioController& radioController, const Debug& debug);
public:
    virtual void loadSlowState(blackboxSlowState_t& blackboxSlowState) override;
    virtual void loadMainState(blackboxMainState_t& blackboxMainState, uint32_t currentTimeUs) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxRcModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroseconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;
    void setUseMessageQueue(bool useMessageQueue) { _useMessageQueue = useMessageQueue; }
private:
    BlackboxMessageQueue& _messageQueue;
    const AHRS& _ahrs;
    const FlightController& _flightController;
    const RadioController& _radioController;
    const ReceiverBase& _receiver;
    const Debug& _debug;
    bool _useMessageQueue {false};
};
