#pragma once

#include <BackchannelStabilizedVehicle.h>

class FlightController;

/*!
Backchannel for Self Balancing Robot.
*/
class BackchannelFlightController : public BackchannelStabilizedVehicle {
public:
    BackchannelFlightController(
        const base_init_t& baseInit,
        uint32_t backchannelID,
        uint32_t telemetryID,
        FlightController& flightController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        SV_Preferences& preferences,
        const TaskBase* mainTask
    );
public:
    virtual bool sendPacket(uint8_t subCommand) override;
protected:
    virtual bool packetControl(const CommandPacketControl& packet) override;
    virtual bool packetSetPID(const CommandPacketSetPID& packet) override;
protected:
    FlightController& _flightController;
};
