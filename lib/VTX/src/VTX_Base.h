#pragma once

#include <TimeMicroseconds.h>
#include <array>
#include <cstdint>

class StreamBuf;


class VTX_Base {
public:
    enum type_e {
        UNSUPPORTED = 0,
        RTC6705     = 1,
        RESERVED    = 2,
        SMART_AUDIO = 3,
        TRAMP       = 4,
        MSP         = 5,
        UNKNOWN     = 0xFF
    };
    enum { MIN_FREQUENCY_MHZ = 5000, MAX_FREQUENCY_MHZ = 5999 };
    
    enum { PIT_MODE_NA, PIT_MODE_OFF, PIT_MODE_ON, PIT_MODE_COUNT };

    // VTX band numbers used for spektrum vtx control
    enum { BAND_COUNT = 5 };
    enum { CHANNEL_COUNT = 8 };
    enum { POWER_LEVEL_COUNT = 8 };
    enum { BAND_USER = 0, BAND_A = 1, BAND_B = 2, BAND_E = 3, BAND_FATSHARK = 4, BAND_RACEBAND = 5, };

    // RTC6705 RF Power index 25 or 200 mW
    enum { 
        RTC6705_POWER_25 = 1,
        RTC6705_POWER_200 = 2
    };
    // SmartAudio "---", 25, 200, 500, 800 mW
    enum {
        SMART_AUDIO_POWER_OFF = 1, // 1 goes to min power, 0 does not do anything (illegal index)
        SMART_AUDIO_POWER_25  = 1,
        SMART_AUDIO_POWER_200 = 2,
        SMART_AUDIO_POWER_500 = 3,
        SMART_AUDIO_POWER_800 = 4,
    };
    // Tramp "---", 25, 100, 200, 400, 600 mW
    enum {
        TRAMP_POWER_OFF = 1, // 1 goes to min power, 0 does not do anything (illegal index)
        TRAMP_POWER_25 = 1,
        TRAMP_POWER_100 = 2,
        TRAMP_POWER_200 = 3,
        TRAMP_POWER_400 = 4,
        TRAMP_POWER_600 = 5,
    };
public:
    enum low_power_disarm_e {
        LOW_POWER_DISARM_OFF = 0,
        LOW_POWER_DISARM_ALWAYS,
        LOW_POWER_DISARM_UNTIL_FIRST_ARM, // Set low power until arming for the first time
    };
    struct config_t {
        uint16_t frequencyMHz;          // sets freq in MHz if band=0
        uint16_t pitModeFrequencyMHz;   // sets out-of-range pitmode frequency
        uint8_t band;           // 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
        uint8_t channel;        // 1-8
        uint8_t power;          // 0 = lowest
        uint8_t lowPowerDisarm; // min power while disarmed, from vtxLowerPowerDisarm_e
        uint8_t softserialAlt;  // prepend 0xff before sending frame even with SOFTSERIAL
    };
public:
    virtual void process(timeUs32_t currentTimeUs) { (void)currentTimeUs; }
    type_e getDeviceType() const { return _type; }
    virtual bool isReady() const { return true; }

    virtual void setBandAndChannel(uint8_t band, uint8_t channel) { _band = band; _channel = channel; }
    virtual bool getBandAndChannel(uint8_t& band, uint8_t& channel) const { band = _band; channel = _channel; return true; }

    virtual void setPowerByIndex(uint8_t index);
    virtual bool getPowerIndex(uint8_t& powerIndex) const { powerIndex = _powerIndex; return true; }
    virtual uint8_t getPowerLevels(uint16_t* levels, uint16_t* powers) const { (void)levels; (void)powers; return 0; }

    virtual void setFrequency(uint16_t frequencyMHz) { (void)frequencyMHz; }
    virtual bool getFrequency(uint16_t& frequencyMHz) const { frequencyMHz = 0; return true; }

    virtual void setPitMode(uint8_t pitMode) { _pitMode = pitMode; }
    virtual bool getStatus(unsigned* status) const { (void)status; return false; }

    virtual void serializeCustomDeviceStatus(StreamBuf& dst) { (void)dst; }
    void lookupBandChannel(uint8_t& band, uint8_t& channel, uint16_t frequencyMHz);
    static uint16_t lookupFrequency(uint8_t band, uint8_t channel);
    bool lookupPowerValue(size_t index, uint16_t& powerValue) const;
protected:
    type_e _type;
    uint16_t _powerValue {};
    bool _configChanged {false};
    uint8_t _powerLevelCount {};
    uint8_t _pitMode {};
    uint8_t _powerIndex {};
    uint8_t _band {};
    uint8_t _channel {};
    uint8_t _region {};
public:
    static const std::array<const char *, BAND_COUNT + 1> BandNames;
    static const std::array<char, BAND_COUNT + 1> BandLetters;
    static const std::array<const char *, CHANNEL_COUNT + 1> ChannelNames;
    static const std::array<const char * const, PIT_MODE_COUNT> PitModeNames;
    static const std::array <uint8_t, POWER_LEVEL_COUNT> PowerIndexTramp;
    static const std::array <uint8_t, POWER_LEVEL_COUNT> PowerIndexRTC670;
    static const std::array <uint8_t, POWER_LEVEL_COUNT> PowerIndexSmartAudio;
    static const uint16_t Frequencies[VTX_Base::BAND_COUNT][CHANNEL_COUNT];
};
