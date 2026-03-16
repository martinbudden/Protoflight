#pragma once

#include <array>
#include <time_microseconds.h>

struct vtx_config_t {
    uint16_t frequency_mhz;          // sets freq in MHz if band=0
    uint16_t pit_mode_frequency_mhz;   // sets out-of-range pit mode frequency
    uint8_t band;           // 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
    uint8_t channel;        // 1-8
    uint8_t power;          // 0 = lowest
    uint8_t lowPowerDisarm; // min power while disarmed, from vtxLowerPowerDisarm_e
    uint8_t softserialAlt;  // prepend 0xff before sending frame even with SOFTSERIAL
};


class VTX
{
public:
    VTX() = default;
private:
    // VTX is not copyable or moveable
    VTX(const VTX&) = delete;
    VTX& operator=(const VTX&) = delete;
    VTX(VTX&&) = delete;
    VTX& operator=(VTX&&) = delete;
public:
    // RTC6705 does not support bands and channels, only frequencies.
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
    static constexpr uint32_t STATUS_PIT_MODE = 0x01;
    static constexpr uint32_t STATUS_LOCKED = 0x02;

    // VTX band numbers used for spektrum vtx control
    enum { BAND_COUNT = 5 };
    enum { CHANNEL_COUNT = 8 };
    enum { POWER_LEVEL_COUNT = 8 };
    enum { BAND_USER = 0, BAND_A = 1, BAND_B = 2, BAND_E = 3, BAND_FATSHARK = 4, BAND_RACEBAND = 5, };
    // check value for MSP_SET_VTX_CONFIG to determine if value is encoded
    // band/channel or frequency in MHz (3 bits for band and 3 bits for channel)
    static constexpr uint16_t MSP_BAND_CHANNEL_CHECK_VALUE =  (0x07 << 3) + 0x07;

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
    enum low_power_disarm_e {
        LOW_POWER_DISARM_OFF = 0,
        LOW_POWER_DISARM_ALWAYS,
        LOW_POWER_DISARM_UNTIL_FIRST_ARM, // Set low power until arming for the first time
    };
public:
    const vtx_config_t& get_config() const { return _config; }
    void set_config(const vtx_config_t& config);

    virtual void process(time_us32_t current_time_us) { (void)current_time_us; }
    type_e get_device_type() const { return _type; }
    virtual bool is_ready() const { return true; }

    virtual void set_band_and_channel(uint8_t band, uint8_t channel) { _band = band; _channel = channel; }
    virtual bool get_band_and_channel(uint8_t& band, uint8_t& channel) const { band = _band; channel = _channel; return true; }

    virtual void set_power_by_index(uint8_t index);
    virtual bool get_power_index(uint8_t& power_index) const { power_index = _power_index; return true; }
    uint8_t get_power_level_count() const { return _power_level_count; }
    virtual uint8_t get_power_levels(uint16_t* levels, uint16_t* powers) const { (void)levels; (void)powers; return 0; }

    virtual void set_frequency(uint16_t frequency_mhz) { (void)frequency_mhz; }
    virtual bool get_frequency(uint16_t& frequency_mhz) const { frequency_mhz = 0; return true; }

    virtual void set_pit_mode(uint8_t pit_mode) { _pit_mode = pit_mode; }
    virtual bool get_status(uint32_t& status) const { status = 0; return false; }

    void lookup_band_channel(uint8_t& band, uint8_t& channel, uint16_t frequency_mhz) const;
    static uint16_t lookup_frequency(uint8_t band, uint8_t channel);
    bool lookup_power_value(size_t index, uint16_t& power_value) const;
    static const char* lookup_band_name(uint8_t band);
    static char lookup_band_letter(uint8_t band);
    static const char* lookup_channel_name(uint8_t channel);
    const char* lookup_power_name(uint8_t power) const;
protected:
    type_e _type;
    vtx_config_t _config;
    uint16_t _power_value {};
    bool _config_changed {false};
    uint8_t _power_level_count {};
    uint8_t _pit_mode {};
    uint8_t _power_index {};
    uint8_t _band {};
    uint8_t _channel {};
    uint8_t _region {};
public:
    static const std::array <uint8_t, POWER_LEVEL_COUNT> _power_indexTramp;
    static const std::array <uint8_t, POWER_LEVEL_COUNT> _power_indexRTC670;
    static const std::array <uint8_t, POWER_LEVEL_COUNT> _power_indexSmartAudio;
    static constexpr uint16_t Frequencies[BAND_COUNT][CHANNEL_COUNT] { // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
        { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boscam A
        { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
        { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, // Boscam E
        { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
        { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
    };
   static constexpr std::array<const char*, BAND_COUNT> BAND_NAMES = {
        "BOSCAM A",
        "BOSCAM B",
        "BOSCAM E",
        "FATSHARK",
        "RACEBAND",
    };
    static constexpr std::array<char, BAND_COUNT> BAND_LETTERS { 'A', 'B', 'E', 'F', 'R' };
    static constexpr std::array<const char*, CHANNEL_COUNT> CHANNEL_NAMES = {
        "1", "2", "3", "4", "5", "6", "7", "8",
    };
};
