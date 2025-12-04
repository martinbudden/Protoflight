#pragma once

#include <array>


namespace LOOKUP_TABLES {

    enum { THROTTLE_LIMIT_NAMES_COUNT = 3 };
    constexpr std::array<const char * const, THROTTLE_LIMIT_NAMES_COUNT> throttleLimitTypeNames { "OFF", "SCALE", "CLIP" };

    enum { PID_TUNING_MODES_COUNT = 2 };
    constexpr std::array<const char * const, PID_TUNING_MODES_COUNT> PID_TuningModes { "STANDARD", "SIMPLIFIED" };

    enum { OFF_ON_COUNT = 2 };
    constexpr std::array<const char * const, OFF_ON_COUNT> offOn { "OFF", "ON" };

    enum { FAILSAFE_PROCEDURE_COUNT =  3 };
    constexpr std::array<const char * const, FAILSAFE_PROCEDURE_COUNT> failsafeProcedureNames { "AUTO-LAND", "DROP", "GPS-RESCUE" };

    enum { LOWPASS_FILTER_TYPES_COUNT = 4 };
    constexpr std::array<const char * const, LOWPASS_FILTER_TYPES_COUNT> lowpassFilterTypes { "PT1", "BIQUAD", "PT2", "PT3" };

    enum { VIDEO_SYSTEMS_COUNT = 4 };
    constexpr std::array<const char * const, VIDEO_SYSTEMS_COUNT> videoSystems { "AUTO", "PAL", "NTSC", "HD" };

} // END LOOKUP_TABLES