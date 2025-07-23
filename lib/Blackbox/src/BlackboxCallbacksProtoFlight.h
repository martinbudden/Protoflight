#pragma once

#include "BlackboxCallbacksBase.h"

class AHRS;
class FlightController;
class RadioController;
class ReceiverBase;


class BlackboxCallbacksProtoFlight : public BlackboxCallbacksBase {
public:
    BlackboxCallbacksProtoFlight(AHRS& ahrs, FlightController& flightController, RadioController& radioController, ReceiverBase& receiver) :
        _ahrs(ahrs),
        _flightController(flightController),
        _radioController(radioController),
        _receiver(receiver)
        {}
public:
    virtual void loadSlowStateFromFlightController(blackboxSlowState_t& blackboxSlowState) override;
    //! Fill the current state of the blackbox using values read from the flight controller
    virtual void loadMainStateFromFlightController(blackboxMainState_t& blackboxMainState) override;
    virtual void loadMainStateFromFlightController(blackboxMainState_t& blackboxMainState, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxRcModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroSeconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;
private:
    AHRS& _ahrs;
    FlightController& _flightController;
    RadioController& _radioController;
    ReceiverBase& _receiver;
};
