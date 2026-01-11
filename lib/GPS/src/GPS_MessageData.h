#pragma once

#include "GeographicCoordinate.h"
#include <cstdint>


struct gps_message_data_t {
    static constexpr uint8_t FIX_HOME = 0x01;
    static constexpr uint8_t FIX = 0x02;
    static constexpr uint8_t FIX_EVER = 0x04;

    int32_t longitude_degrees1E7;
    int32_t latitude_degrees1E7;
    int32_t altitude_cm;
    float distanceToHomeMeters;
    float bearingToHomeDegrees;
    float distanceFlownMeters;
    uint32_t timeOfWeek_ms;
    int16_t velocityNorth_cmps;
    int16_t velocityEast_cmps;
    int16_t velocityDown_cmps;
    int16_t speed3d_cmps;
    int16_t groundSpeed_cmps;
    int16_t heading_deciDegrees;
    uint16_t dilutionOfPrecisionPositional;
    uint8_t satelliteCount;
    uint8_t fix;
    uint8_t isHealthy;
};
