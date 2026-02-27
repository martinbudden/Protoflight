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
    float distance_to_home_meters;
    float bearing_to_home_degrees;
    float distance_flown_meters;
    uint32_t time_of_week_ms;
    int16_t velocity_north_cmps;
    int16_t velocity_east_cmps;
    int16_t velocity_down_cmps;
    int16_t speed3d_cmps;
    int16_t ground_speed_cmps;
    int16_t heading_deci_degrees;
    uint16_t dilution_of_precision_positional;
    uint8_t satellite_count;
    uint8_t fix;
    uint8_t is_healthy;
    uint8_t update;
};
