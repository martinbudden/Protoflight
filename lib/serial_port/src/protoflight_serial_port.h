#pragma once

#include <cstdint>


class ProtoFlightSerialPort {
public:
    struct config_t {
        uint32_t functionMask;
        int8_t identifier;
        uint8_t msp_baudrate_index;
        uint8_t gps_baudrate_index;
        uint8_t blackbox_baudrate_index;
        uint8_t telemetry_baudrate_index; // not used for all telemetry systems, e.g. HoTT only works at 19200.
    };
};
