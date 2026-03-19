#pragma once

#include <backchannel_base.h>

class Ahrs;
class AhrsMessageQueue;
class Debug;
class FlightController;
class MotorMixerBase;
class MspBase;
class NonVolatileStorage;
class ReceiverBase;
class TaskBase;
struct msp_context_t;

struct backchannel_context_t {
    Ahrs& ahrs;
    const AhrsMessageQueue& ahrs_message_queue;
    FlightController& flight_controller;
    MotorMixerBase& motor_mixer;
    ReceiverBase& receiver;
    TaskBase* main_task;
    MspBase* msp;
    msp_context_t* msp_context;
    Debug& debug;
    NonVolatileStorage& nvs;
};


/*!
Backchannel for Flight Controller.
*/
class BackchannelFlightController : public BackchannelBase {
public:
    BackchannelFlightController(
        BackchannelTransceiverBase& backchannel_transceiver,
        const uint8_t* backchannel_mac_address,
        const uint8_t* my_mac_address
    );
public:
    virtual bool send_packet(backchannel_context_t& ctx, uint8_t sub_command) override;
protected:
    virtual bool packet_set_offset(backchannel_context_t& ctx, const CommandPacketSetOffset& packet) override;
    virtual bool packet_control(backchannel_context_t& ctx, const CommandPacketControl& packet) override;
    virtual bool packet_set_pid(backchannel_context_t& ctx, const CommandPacketSetPid& packet) override;
};
