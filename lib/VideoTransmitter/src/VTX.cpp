#include "VTX.h"


void VTX::setConfig(const config_t& config)
{
    _config = config;
}

/*!
Converts frequencyMHz to band and channel values.
If frequency not found then band and channel are set to 0.
*/
void VTX::lookupBandChannel(uint8_t& band, uint8_t& channel, uint16_t frequencyMHz)
{
    if (frequencyMHz == 5880) {
        // 5880Mhz returns Raceband 7 rather than Fatshark 8.
        band = BAND_RACEBAND;
        channel = 7;
        return;
    }
    for (band = 0; band < BAND_COUNT ; ++band) {
        for (channel = 0 ; channel < CHANNEL_COUNT ; ++channel) {
            if (Frequencies[band][channel] == frequencyMHz) { // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
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
uint16_t VTX::lookupFrequency(uint8_t band, uint8_t channel)
{
    if (band > 0 && band <= BAND_COUNT && channel > 0 && channel <= CHANNEL_COUNT) {
        return Frequencies[band - 1][channel - 1]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }

    return 0;
}

bool VTX::lookupPowerValue(size_t index, uint16_t& powerValue) const
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

    const std::array <uint8_t, VTX::POWER_LEVEL_COUNT>& powerValues = 
        (_type == RTC6705) ? PowerIndexRTC670 :
        (_type == SMART_AUDIO) ? PowerIndexSmartAudio : PowerIndexTramp;


    if (index > 0 && index <= _powerLevelCount) {
        powerValue = powerValues[index - 1];
        return true;
    }
    return false;
}

void VTX::setPowerByIndex(uint8_t index)
{
    uint16_t powerValue {};

    if (lookupPowerValue(index, powerValue)) {
        if (powerValue != _powerValue) {
            _configChanged = true;
        }
        _powerValue = powerValue;
    }
}


// Tramp "---", 25, 200, 400. 600 mW
const std::array <uint8_t, VTX::POWER_LEVEL_COUNT> VTX::PowerIndexTramp {
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
const std::array <uint8_t, VTX::POWER_LEVEL_COUNT> VTX::PowerIndexRTC670 {
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
const std::array <uint8_t, VTX::POWER_LEVEL_COUNT> VTX::PowerIndexSmartAudio {
    SMART_AUDIO_POWER_OFF,  // Off
    SMART_AUDIO_POWER_OFF,  //   1 -  14mW
    SMART_AUDIO_POWER_25,   //  15 -  25mW
    SMART_AUDIO_POWER_25,   //  26 -  99mW

    SMART_AUDIO_POWER_200,  // 100 - 299mW
    SMART_AUDIO_POWER_500,  // 300 - 600mW
    SMART_AUDIO_POWER_800,  // 601 - max
    SMART_AUDIO_POWER_200   // Manual
};
