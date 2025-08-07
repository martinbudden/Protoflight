#pragma once

#include <Blackbox.h>
#include <FlightController.h>

class IMU_Filters;
class RadioController;

/*!
Class to write out the Blackbox header, written in blackboxWriteSysinfo()
*/
class BlackboxProtoFlight : public Blackbox {
public:
    BlackboxProtoFlight(BlackboxCallbacksBase& callbacks, BlackboxMessageQueueBase& messageQueue, BlackboxSerialDevice& serialDevice, const FlightController& flightController, const RadioController& radioController, const IMU_Filters& imuFilters) :
        Blackbox(flightController.getTaskIntervalMicroSeconds(), callbacks, messageQueue, serialDevice),
        _flightController(flightController),
        _radioController(radioController),
        _imuFilters(imuFilters)
        {}
public:
    virtual Blackbox::write_e writeSystemInformation() override;
private:
    const FlightController& _flightController;
    const RadioController& _radioController;
    const IMU_Filters& _imuFilters;
};
