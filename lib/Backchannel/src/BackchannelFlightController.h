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
struct msp_parameter_group_t;

struct backchannel_parameter_group_t {
    Ahrs& ahrs;
    const AhrsMessageQueue& ahrs_message_queue;
    FlightController& flight_controller;
    MotorMixerBase& motor_mixer;
    ReceiverBase& receiver;
    TaskBase* main_task;
    MspBase* msp;
    msp_parameter_group_t* msp_parameter_group;
    Debug& debug;
    NonVolatileStorage& nvs;
};


/*!
Backchannel for Self Balancing Robot.
*/
class BackchannelFlightController : public BackchannelBase {
public:
    BackchannelFlightController(
        BackchannelTransceiverBase& backchannelTransceiver,
        const uint8_t* backchannel_mac_address,
        const uint8_t* my_mac_address
    );
public:
    virtual bool send_packet(backchannel_parameter_group_t& pg, uint8_t sub_command) override;
protected:
    virtual bool packet_set_offset(backchannel_parameter_group_t& pg, const CommandPacketSetOffset& packet) override;
    virtual bool packet_control(backchannel_parameter_group_t& pg, const CommandPacketControl& packet) override;
    virtual bool packet_set_pid(backchannel_parameter_group_t& pg, const CommandPacketSetPid& packet) override;
};
