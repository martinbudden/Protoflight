#include "VTX_Base.h"


const std::array<const char *, VTX_Base::BAND_COUNT + 1> VTX_Base::BandNames
{{
    "--------",
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
}};

const std::array<char, VTX_Base::BAND_COUNT + 1> VTX_Base::BandLetters { '-', 'A', 'B', 'E', 'F', 'R' };
const std::array<const char *, VTX_Base::CHANNEL_COUNT + 1> VTX_Base::ChannelNames { "-", "1", "2", "3", "4", "5", "6", "7", "8" };
const std::array<const char * const, VTX_Base::PIT_MODE_COUNT> VTX_Base::PitModeNames { "---", "OFF", "ON " };

const uint16_t VTX_Base::Frequencies[VTX_Base::BAND_COUNT][VTX_Base::CHANNEL_COUNT] { // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boscam A
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, // Boscam E
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
};

// Tramp "---", 25, 200, 400. 600 mW
const std::array <uint8_t, VTX_Base::POWER_LEVEL_COUNT> VTX_Base::PowerIndexTramp {
                        // Spektrum Spec    Tx menu  Tx sends   To VTX    Watt
    TRAMP_POWER_OFF,    //         Off      INHIBIT         0        0     -
    TRAMP_POWER_OFF,    //   1 -  14mW            -         -        -     -
    TRAMP_POWER_25,     //  15 -  25mW   15 -  25mW         2        1    25mW
    TRAMP_POWER_100,    //  26 -  99mW   26 -  99mW         3        2   100mW Slightly outside range

    TRAMP_POWER_200,    // 100 - 299mW  100 - 200mW         4        3   200mW
    TRAMP_POWER_400,    // 300 - 600mW  300 - 600mW         5        4   400mW
    TRAMP_POWER_600,    // 601 - max    601+ mW             6        5   600mW Slightly outside range
    TRAMP_POWER_200     // Manual               -           -        -     -
};

// RTC6705 "---", 25 or 200 mW
const std::array <uint8_t, VTX_Base::POWER_LEVEL_COUNT> VTX_Base::PowerIndexRTC670 {
    RTC6705_POWER_25,   // Off
    RTC6705_POWER_25,   //   1 -  14mW
    RTC6705_POWER_25,   //  15 -  25mW
    RTC6705_POWER_25,   //  26 -  99mW

    RTC6705_POWER_200,  // 100 - 299mW
    RTC6705_POWER_200,  // 300 - 600mW
    RTC6705_POWER_200,  // 601 - max
    RTC6705_POWER_200   // Manual
};

// SmartAudio "---", 25, 200, 500. 800 mW
const std::array <uint8_t, VTX_Base::POWER_LEVEL_COUNT> VTX_Base::PowerIndexSmartAudio {
    SMART_AUDIO_POWER_OFF,  // Off
    SMART_AUDIO_POWER_OFF,  //   1 -  14mW
    SMART_AUDIO_POWER_25,   //  15 -  25mW
    SMART_AUDIO_POWER_25,   //  26 -  99mW

    SMART_AUDIO_POWER_200,  // 100 - 299mW
    SMART_AUDIO_POWER_500,  // 300 - 600mW
    SMART_AUDIO_POWER_800,  // 601 - max
    SMART_AUDIO_POWER_200   // Manual
};
