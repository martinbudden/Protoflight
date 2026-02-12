#pragma once

#include <BlackboxCallbacksBase.h>

class AHRS_MessageQueue;
class Cockpit;
class Debug;
class FlightController;
class GPS;
class RcModes;
class ReceiverBase;


class BlackboxCallbacks : public BlackboxCallbacksBase {
public:
    BlackboxCallbacks(const AHRS_MessageQueue& messageQueue, const FlightController& flightController, const Cockpit& cockpit, const ReceiverBase& receiver, const RcModes& rc_modes, const Debug& debug, const GPS* gps);
public:
    virtual void loadSlowState(blackbox_slow_state_t& blackboxSlowState) override;
    virtual void loadMainState(blackbox_main_state_t& blackboxMainState, uint32_t currentTimeUs) override;
    virtual void loadGPS_State(blackbox_gps_state_t& gpsState) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxModeActive() const override;
    virtual bool isBlackboxEraseModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroseconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;
    virtual void beep() const override;
private:
    const AHRS_MessageQueue& _messageQueue;
    const FlightController& _flightController;
    const Cockpit& _cockpit;
    const RcModes& _rc_modes;
    const ReceiverBase& _receiver;
    const Debug& _debug;
    const GPS* _gps;
};
