#pragma once

#include <array>
#include <cstdint>


struct rates_t {
    enum { ROLL = 0, PITCH = 1, YAW = 2, AXIS_COUNT = 3 };
    enum { LIMIT_MAX = 1998 };
    enum { RC_RATES_MAX = 255 };
    enum { RC_EXPOS_MAX = 100 };
    enum { THROTTLE_MAX = 100 };
    enum type_e { TYPE_BETAFLIGHT = 0, TYPE_RACEFLIGHT, TYPE_KISS, TYPE_ACTUAL, TYPE_QUICK, TYPE_COUNT } ;
    enum throttle_limit_type_e { THROTTLE_LIMIT_TYPE_OFF = 0, THROTTLE_LIMIT_TYPE_SCALE, THROTTLE_LIMIT_TYPE_CLIP, THROTTLE_LIMIT_TYPE_COUNT };

    std::array<uint16_t, AXIS_COUNT> rateLimits;
    std::array<uint8_t, AXIS_COUNT> rcRates; // center sensitivity
    std::array<uint8_t, AXIS_COUNT> rcExpos; // movement sensitivity, nonlinear
    std::array<uint8_t, AXIS_COUNT> rates; // movement sensitivity, linear
    uint8_t throttleMidpoint; // not used
    uint8_t throttleExpo;
    uint8_t throttleLimitType; // not used
    uint8_t throttleLimitPercent; // Sets the maximum pilot commanded throttle limit
    //uint8_t ratesType; // not used
};
