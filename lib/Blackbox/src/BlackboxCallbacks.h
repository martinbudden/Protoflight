#pragma once

#include "BlackboxCallbacksBase.h"

class AHRS;
class BlackboxMessageQueue;
class FlightController;
class RadioControllerBase;
class ReceiverBase;


class BlackboxCallbacks : public BlackboxCallbacksBase {
public:
    BlackboxCallbacks(BlackboxMessageQueue& messageQueue, AHRS& ahrs, FlightController& flightController, RadioControllerBase& radioController, ReceiverBase& receiver) :
        _messageQueue(messageQueue),
        _ahrs(ahrs),
        _flightController(flightController),
        _radioController(radioController),
        _receiver(receiver)
        {}
public:
    virtual void loadSlowState(blackboxSlowState_t& blackboxSlowState) override;
    virtual void loadMainState(blackboxMainState_t& blackboxMainState, uint32_t currentTimeUs) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxRcModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroSeconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;
private:
    BlackboxMessageQueue& _messageQueue;
    AHRS& _ahrs;
    FlightController& _flightController;
    RadioControllerBase& _radioController;
    ReceiverBase& _receiver;
};
