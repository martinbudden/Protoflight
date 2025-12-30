#pragma once

#include <algorithm>
#include <array>
#include <cstdint>


class RX {
public:
    enum {
        MAX_SUPPORTED_RC_CHANNEL_COUNT = 18,
        MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT = 8,
        STICK_CHANNEL_COUNT = 4,
        MAX_AUX_CHANNEL_COUNT = MAX_SUPPORTED_RC_CHANNEL_COUNT - STICK_CHANNEL_COUNT
    };

    static constexpr uint16_t PULSE_WIDTH_MIN_MICROSECONDS = 885;
    static constexpr uint16_t PULSE_WIDTH_MAX_MICROSECONDS = 2115;
    static constexpr uint16_t PULSE_WIDTH_MID_MICROSECONDS = 1500;

    static constexpr uint16_t PWM_RANGE_MIN = 1000;
    static constexpr uint16_t PWM_RANGE_MAX = 2000;
    static constexpr uint16_t PWM_RANGE = (PWM_RANGE_MAX - PWM_RANGE_MIN);
    static constexpr uint16_t PWM_RANGE_MIDDLE = (PWM_RANGE_MIN + (PWM_RANGE / 2));
    static constexpr uint16_t PWM_PULSE_MIN = 750;       // minimum PWM pulse width which is considered valid
    static constexpr uint16_t PWM_PULSE_MAX = 2250;      // maximum PWM pulse width which is considered valid
    static constexpr uint16_t MAX_RXFAIL_RANGE_STEP = (PWM_PULSE_MAX - PWM_PULSE_MIN) / 25;

    static uint16_t failStepToChannelValue(uint8_t step) { return (PWM_PULSE_MIN + static_cast<uint16_t>(25 * step)); }
    static uint8_t channelValueToFailStep(uint16_t channelValue) { return static_cast<uint8_t>((std::clamp(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25); }

    enum provider_e {
        PROVIDER_NONE = 0,
        PROVIDER_PARALLEL_PWM,
        PROVIDER_PPM,
        PROVIDER_SERIAL,
        PROVIDER_MSP,
        PROVIDER_SPI,
        PROVIDER_UDP,
    };
    enum frame_state_e {
        FRAME_PENDING = 0,
        FRAME_COMPLETE = 0x01,
        RX_FRAME_FAILSAFE = 0x02,
        RX_FRAME_PROCESSING_REQUIRED = 0x04,
        RX_FRAME_DROPPED = 0x08
    };
    enum serial_type_e {
        SERIAL_NONE = 0,
        SERIAL_SPEKTRUM2048 = 1,
        SERIAL_SBUS = 2,
        SERIAL_SUMD = 3,
        SERIAL_SUMH = 4,
        SERIAL_XBUS_MODE_B = 5,
        SERIAL_XBUS_MODE_B_RJ01 = 6,
        SERIAL_IBUS = 7,
        SERIAL_JETIEXBUS = 8,
        SERIAL_CRSF = 9,
        SERIAL_SRXL = 10,
        SERIAL_TARGET_CUSTOM = 11,
        SERIAL_FPORT = 12,
        SERIAL_SRXL2 = 13,
        SERIAL_GHST = 14,
        SERIAL_SPEKTRUM1024 = 15,
        SERIAL_MAVLINK = 16,
    };
    enum rssi_source_e {
        RSSI_SOURCE_NONE = 0,
        RSSI_SOURCE_ADC,
        RSSI_SOURCE_RX_CHANNEL,
        RSSI_SOURCE_RX_PROTOCOL,
        RSSI_SOURCE_MSP,
        RSSI_SOURCE_FRAME_ERRORS,
        RSSI_SOURCE_RX_PROTOCOL_CRSF,
        RSSI_SOURCE_RX_PROTOCOL_MAVLINK,
    };
    enum { RSSI_MAX_VALUE = 1023 };
    enum link_quality_source_e {
        LQ_SOURCE_NONE = 0,
        LQ_SOURCE_RX_PROTOCOL_CRSF,
        LQ_SOURCE_RX_PROTOCOL_GHST,
        LQ_SOURCE_RX_PROTOCOL_MAVLINK,
    };
    enum { LINK_QUALITY_MAX_VALUE = 1023 };
    enum { MAPPABLE_CHANNEL_COUNT = 8 };
    struct config_t {
        // std::array<uint8_t, MAPPABLE_CHANNEL_COUNT> rc_map;  // mapping of radio channels to internal RPYTA+ order
        uint8_t serial_rx_provider;
        uint8_t serial_rx_inverted; // invert the serial RX protocol compared to its default setting
        uint8_t half_duplex;        // allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
        uint8_t rssi_channel;
        uint8_t rssi_scale;
        uint8_t rssi_invert;
        int8_t rssi_offset;         // offset applied to the RSSI value
        uint8_t fpvCamAngleDegrees;         // Camera angle to be scaled into rc commands
        uint8_t airModeActivateThreshold;   // Throttle setpoint percent where airmode gets activated
        uint8_t spektrum_sat_bind;  // number of bind pulses for Spektrum satellite receivers
        uint16_t mid_rc;            // Some radios have not a neutral point centered on 1500. can be changed here
        uint16_t min_check;         // minimum rc end
        uint16_t max_check;         // maximum rc end
        uint16_t rx_min_usec;
        uint16_t rx_max_usec;
    };
    enum failsafe_channel_type_e {
        FAILSAFE_TYPE_FLIGHT = 0,
        FAILSAFE_TYPE_AUX,
        FAILSAFE_TYPE_COUNT
    };
    enum failsafe_channel_mode_e {
        FAILSAFE_MODE_AUTO = 0,
        FAILSAFE_MODE_HOLD,
        FAILSAFE_MODE_SET,
        FAILSAFE_MODE_INVALID,
        FAILSAFE_MODE_COUNT = 3
    };
    struct failsafe_channel_config_t {
        uint8_t mode; // failsafe_channel_mode_e
        uint8_t step;
    };
    typedef std::array<RX::failsafe_channel_config_t, RX::MAX_SUPPORTED_RC_CHANNEL_COUNT> failsafe_channel_configs_t;
    struct channel_range_config_t {
        uint16_t min;
        uint16_t max;
    };
    typedef std::array<RX::channel_range_config_t, RX::STICK_CHANNEL_COUNT> channel_range_configs_t;
public:
    config_t _config;
    failsafe_channel_configs_t _failsafeChannelConfigs;
    channel_range_configs_t _channelRangeConfigs;
};
