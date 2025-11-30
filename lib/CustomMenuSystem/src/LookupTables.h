#pragma once

#include <array>


namespace LOOKUP_TABLES {

    constexpr std::array<const char * const, 3> throttleLimitTypeNames { "OFF", "SCALE", "CLIP" };
    constexpr std::array<const char * const, 3> PID_TuningModes { "STANDARD", "RP", "RPY" };
    constexpr std::array<const char * const, 2> offOn { "OFF", "ON" };
    constexpr std::array<const char * const, 3> failsafeProcedureNames { "AUTO-LAND", "DROP", "GPS-RESCUE" };
    constexpr std::array<const char * const, 4> lowpassFilterTypes { "PT1", "BIQUAD", "PT2", "PT3" };
    constexpr std::array<const char * const, 4> videoSystems { "AUTO", "PAL", "NTSC", "HD" };

} // END LOOKUP_TABLES