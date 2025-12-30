#pragma once

#include <cstdint>


class ProtoFlightSerialPort {
public:
    struct config_t {
        uint32_t functionMask;
        int8_t identifier;
        uint8_t msp_baudrateIndex;
        uint8_t gps_baudrateIndex;
        uint8_t blackbox_baudrateIndex;
        uint8_t telemetry_baudrateIndex; // not used for all telemetry systems, e.g. HoTT only works at 19200.
    };
};