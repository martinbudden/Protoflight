#include "vtx.h"


void VTX::set_config(const vtx_config_t& config)
{
    _config = config;
}

/*!
Converts frequency_mhz to band and channel values.
If frequency not found then band and channel are set to 0.
*/
void VTX::lookup_band_channel(uint8_t& band, uint8_t& channel, uint16_t frequency_mhz) const
{
    if (frequency_mhz == 5880) {
        // 5880Mhz returns Raceband 7 rather than Fatshark 8.
        band = BAND_RACEBAND;
        channel = 7;
        return;
    }
    for (band = 0; band < BAND_COUNT; ++band) {
        for (channel = 0; channel < CHANNEL_COUNT; ++channel) {
            if (Frequencies[band][channel] == frequency_mhz) { // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
                ++band;
                ++channel;
                return;
            }
        }
    }
    band = 0;
    channel = 0;
}

/*!
Converts band and channel values to a frequency (in MHz) value.
band:  Band value (1 to 5).
channel:  Channel value (1 to 8).
Returns frequency value (in MHz), or 0 if band/channel out of range.
*/
uint16_t VTX::lookup_frequency(uint8_t band, uint8_t channel)
{
    if (band > 0 && band <= BAND_COUNT && channel > 0 && channel <= CHANNEL_COUNT) {
        return Frequencies[band - 1][channel - 1]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }

    return 0;
}

bool VTX::lookup_power_value(size_t index, uint16_t& power_value) const
{
    switch (_type) {
    case RTC6705:
        break;
    case SMART_AUDIO:
        break;
    case TRAMP:
        break;
    case MSP:
        return false;
    default:
        return false;
    }

    const std::array <uint8_t, VTX::POWER_LEVEL_COUNT>& power_values =
        (_type == RTC6705) ? _power_indexRTC670 :
        (_type == SMART_AUDIO) ? _power_indexSmartAudio : _power_indexTramp;


    if (index > 0 && index <= _power_level_count) {
        power_value = power_values[index - 1];
        return true;
    }
    return false;
}

void VTX::set_power_by_index(uint8_t index)
{
    uint16_t power_value {};

    if (lookup_power_value(index, power_value)) {
        if (power_value != _power_value) {
            _config_changed = true;
        }
        _power_value = power_value;
    }
}

const char* VTX::lookup_band_name(uint8_t band)
{
    return (band < BAND_COUNT) ? BAND_NAMES[band] : "?";
}

char VTX::lookup_band_letter(uint8_t band)
{
    return (band < BAND_COUNT) ? BAND_LETTERS[band] : '?';
}

const char* VTX::lookup_channel_name(uint8_t channel)
{
    return (channel < CHANNEL_COUNT) ? CHANNEL_NAMES[channel] : "?";
}

const char* VTX::lookup_power_name(uint8_t power_index) const
{
    (void)power_index;

    return "?";
}

// Tramp "---", 25, 200, 400. 600 mW
const std::array <uint8_t, VTX::POWER_LEVEL_COUNT> VTX::_power_indexTramp {
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
const std::array <uint8_t, VTX::POWER_LEVEL_COUNT> VTX::_power_indexRTC670 {
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
const std::array <uint8_t, VTX::POWER_LEVEL_COUNT> VTX::_power_indexSmartAudio {
    SMART_AUDIO_POWER_OFF,  // Off
    SMART_AUDIO_POWER_OFF,  //   1 -  14mW
    SMART_AUDIO_POWER_25,   //  15 -  25mW
    SMART_AUDIO_POWER_25,   //  26 -  99mW

    SMART_AUDIO_POWER_200,  // 100 - 299mW
    SMART_AUDIO_POWER_500,  // 300 - 600mW
    SMART_AUDIO_POWER_800,  // 601 - max
    SMART_AUDIO_POWER_200   // Manual
};
