# pragma once

/*!
Packet definitions of telemetry data specific to Flight Controllers.
*/


#include <array>
#include <cstdint>
#include <pid_controller.h>


#pragma pack(push, 1)
/*!
Packet for the the transmission of PID constants and setpoints, to enable remote tuning.
*/
struct TD_FC_PIDS {
    enum { TYPE = 50 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_FC_PIDS)}; //!< length of whole packet, ie sizeof(TD_FC_PIDS)
    uint8_t sub_type {0};
    uint8_t sequence_number {0};

    enum {
        ROLL_RATE_DPS = 0,
        PITCH_RATE_DPS = 1,
        YAW_RATE_DPS = 2,
        ROLL_ANGLE_DEGREES = 3,
        PITCH_ANGLE_DEGREES = 4,
        ROLL_SIN_ANGLE = 5,
        PITCH_SIN_ANGLE = 6,
        PID_COUNT = 7,
        PID_BEGIN = 0
    };
    struct SPID_t {
        float setpoint;
        pid_constants_t pid;
    };
    std::array<SPID_t, PID_COUNT> spids;
};

#pragma pack(pop)
