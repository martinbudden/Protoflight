#pragma once

#include <Blackbox.h>
#include <FlightController.h>

class RadioController;

/*!
Class to write out the Blackbox header, written in blackboxWriteSysinfo()
*/
class BlackboxProtoFlight : public Blackbox {
public:
    BlackboxProtoFlight(BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serialDevice, FlightController& flightController, RadioController& radioController) :
        Blackbox(flightController.getTaskIntervalMicroSeconds(), callbacks, serialDevice),
        _flightController(flightController),
        _radioController(radioController)
        {}
public:
    virtual Blackbox::write_e writeSystemInformation() override;
private:
    FlightController& _flightController;
    RadioController& _radioController;
};
