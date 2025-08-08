#pragma once

#include <BackchannelStabilizedVehicle.h>

class FlightController;
class NonVolatileStorage;

/*!
Backchannel for Self Balancing Robot.
*/
class BackchannelFlightController : public BackchannelStabilizedVehicle {
public:
    BackchannelFlightController(
        BackchannelTransceiverBase& backchannelTransceiver,
        const uint8_t* backchannelMacAddress,
        const uint8_t* myMacAddress,
        FlightController& flightController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        const TaskBase* mainTask,
        NonVolatileStorage& nonVolatileStorage
    );
public:
    virtual bool sendPacket(uint8_t subCommand) override;
protected:
    virtual bool packetSetOffset(const CommandPacketSetOffset& packet) override;
    virtual bool packetControl(const CommandPacketControl& packet) override;
    virtual bool packetSetPID(const CommandPacketSetPID& packet) override;
protected:
    FlightController& _flightController;
    NonVolatileStorage& _nonVolatileStorage;
};
