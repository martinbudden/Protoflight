#pragma once

#include <Blackbox.h>

class Cockpit;
class FlightController;
class IMU_Filters;

/*!
Class to write out the Blackbox header, written in blackboxWriteSysinfo()
*/
class BlackboxProtoflight : public Blackbox {
public:
    BlackboxProtoflight(uint32_t pidLooptimeUs, BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serialDevice, const FlightController& flightController, const Cockpit& cockpit, const IMU_Filters& imuFilters) :
        Blackbox(pidLooptimeUs, callbacks, serialDevice),
        _flightController(flightController),
        _cockpit(cockpit),
        _imuFilters(imuFilters)
        {}
public:
    virtual Blackbox::write_e writeSystemInformation() override;
private:
    const FlightController& _flightController;
    const Cockpit& _cockpit;
    const IMU_Filters& _imuFilters;
};
