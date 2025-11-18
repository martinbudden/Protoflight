#pragma once

#include "DisplayPortBase.h"
#include "OSD_Elements.h"
#include <TimeMicroseconds.h>
#include <array>
#include <cstdint>

class AHRS_MessageQueue;
class Cockpit;
class Debug;

/*!
On Screen Display.
*/
class OSD {
public:
    OSD(const FlightController& flightController, const Cockpit& cockpit, const AHRS_MessageQueue& ahrsMessageQueue, Debug& debug);
private:
    // OSD is not copyable or moveable
    OSD(const OSD&) = delete;
    OSD& operator=(const OSD&) = delete;
    OSD(OSD&&) = delete;
    OSD& operator=(OSD&&) = delete;
public:
    enum { PROFILE_NAME_LENGTH = 16 };
    enum { RC_CHANNELS_COUNT = 4 };
    enum { LOGO_ROW_COUNT = 4, LOGO_COLUMN_COUNT = 24 };
    enum { SD_ROWS = 16, SD_COLS = 30 };
    enum { HD_ROWS = 20, HD_COLS = 53 };
    enum { FRAMERATE_DEFAULT_HZ = 12 };
    enum { ESC_RPM_ALARM_OFF = -1, ESC_TEMP_ALARM_OFF = 0, ESC_CURRENT_ALARM_OFF = -1 };
    enum { ELEMENT_BUFFER_LENGTH = 32 };

    enum  state_e {
        STATE_INIT,
        STATE_IDLE,
        STATE_CHECK,
        STATE_PROCESS_STATS1,
        STATE_REFRESH_STATS,
        STATE_PROCESS_STATS2,
        STATE_PROCESS_STATS3,
        STATE_UPDATE_ALARMS,
        STATE_REFRESH_PRE_ARM,
        STATE_UPDATE_CANVAS,
        // Elements are handled in two steps, drawing into a buffer, and then sending to the display
        STATE_DRAW_ELEMENT,
        STATE_DISPLAY_ELEMENT,
        STATE_UPDATE_HEARTBEAT,
        STATE_COMMIT,
        STATE_TRANSFER,
        STATE_COUNT
    };
    enum  stats_e {
        STATS_RTC_DATE_TIME,
        STATS_TIMER_1,
        STATS_TIMER_2,
        STATS_MAX_SPEED,
        STATS_MAX_DISTANCE,
        STATS_MIN_BATTERY,
        STATS_END_BATTERY,
        STATS_BATTERY,
        STATS_MIN_RSSI,
        STATS_MAX_CURRENT,
        STATS_USED_MAH,
        STATS_MAX_ALTITUDE,
        STATS_BLACKBOX,
        STATS_BLACKBOX_NUMBER,
        STATS_MAX_G_FORCE,
        STATS_MAX_ESC_TEMP,
        STATS_MAX_ESC_RPM,
        STATS_MIN_LINK_QUALITY,
        STATS_FLIGHT_DISTANCE,
        STATS_MAX_FFT,
        STATS_TOTAL_FLIGHTS,
        STATS_TOTAL_TIME,
        STATS_TOTAL_DIST,
        STATS_MIN_RSSI_DBM,
        STATS_WATT_HOURS_DRAWN,
        STATS_MIN_RSNR,
        STATS_BEST_3_CONSEC_LAPS,
        STATS_BEST_LAP,
        STATS_FULL_THROTTLE_TIME,
        STATS_FULL_THROTTLE_COUNTER,
        STATS_AVG_THROTTLE,
        STATS_COUNT // MUST BE LAST
    };
    enum refresh_stats_state_e {
        REFRESH_STATS_STATE_INITIAL_CLEAR_SCREEN = 0,
        REFRESH_STATS_STATE_COUNT_STATS,
        REFRESH_STATS_STATE_CLEAR_SCREEN,
        REFRESH_STATS_STATE_RENDER_STATS,
    };

    enum logo_on_arming_e { LOGO_ARMING_OFF, LOGO_ARMING_ON, LOGO_ARMING_FIRST };
    enum timer_e {
        TIMER_1,
        TIMER_2,
        TIMER_COUNT
    };
    enum timer_source_e {
        TIMER_SRC_ON,
        TIMER_SRC_TOTAL_ARMED,
        TIMER_SRC_LAST_ARMED,
        TIMER_SRC_ON_OR_ARMED,
        TIMER_SRC_LAUNCH_TIME,
        TIMER_SRC_COUNT
    };
    enum timer_precision_e {
        TIMER_PREC_SECOND,
        TIMER_PREC_HUNDREDTHS,
        TIMER_PREC_TENTHS,
        TIMER_PREC_COUNT
    };
    enum warnings_flags_e {
        WARNING_ARMING_DISABLE,
        WARNING_BATTERY_NOT_FULL,
        WARNING_BATTERY_WARNING,
        WARNING_BATTERY_CRITICAL,
        WARNING_VISUAL_BEEPER,
        WARNING_CRASHFLIP,
        WARNING_ESC_FAIL,
        WARNING_CORE_TEMPERATURE,
        WARNING_RC_SMOOTHING,
        WARNING_FAIL_SAFE,
        WARNING_LAUNCH_CONTROL,
        WARNING_GPS_RESCUE_UNAVAILABLE,
        WARNING_GPS_RESCUE_DISABLED,
        WARNING_RSSI,
        WARNING_LINK_QUALITY,
        WARNING_RSSI_DBM,
        WARNING_OVER_CAP,
        WARNING_RSNR,
        WARNING_LOAD,
        WARNING_POSHOLD_FAILED,
        WARNING_COUNT // MUST BE LAST
    };

public:
    struct config_t {
        char profile[OSD_Elements::PROFILE_COUNT][PROFILE_NAME_LENGTH + 2]; // extra byte for zero terminator and extra byte to even-align
        int8_t rcChannels[RC_CHANNELS_COUNT];   // RC channel values to display, -1 if none
        uint16_t timers[TIMER_COUNT];

        uint32_t enabled_warnings;
        uint32_t enabled_stats;

        uint16_t framerate_hz;

        uint16_t cap_alarm;
        uint16_t alt_alarm;
        uint16_t link_quality_alarm;
        int16_t rssi_dbm_alarm;
        int16_t rsnr_alarm;
        uint16_t distance_alarm;
        int16_t esc_rpm_alarm;
        int16_t esc_current_alarm;
        uint8_t esc_temp_alarm;
        uint8_t core_temp_alarm;
        uint8_t rssi_alarm;

        uint8_t units;

        uint16_t aux_scale;
        uint8_t aux_channel;
        uint8_t aux_symbol;

        uint8_t logo_on_arming;
        uint8_t logo_on_arming_duration;        // display duration in 0.1s units
        uint8_t arming_logo_attribute;          // font attribute to use to display the logo on arming

        uint8_t ahMaxPitch;
        uint8_t ahMaxRoll;
        uint8_t ahInvert;
        uint8_t osdProfileIndex;
        uint8_t overlay_radio_mode;
        uint8_t gps_sats_show_pdop;
        uint8_t camera_frame_width;
        uint8_t camera_frame_height;
        uint8_t cms_background_type;            // whether the CMS background is transparent or opaque
        uint8_t stats_show_cell_value;
        uint8_t osd_craftname_messages;         // Insert LQ/RSSI-dBm and warnings into CraftName
        uint8_t display_port_device_type;
        uint8_t canvas_column_count;
        uint8_t canvas_row_count;
        uint8_t osd_use_quick_menu;
        uint8_t osd_show_spec_prearm;
    };
    struct statsConfig_t {
        uint32_t stats_total_flights;
        uint32_t stats_total_time_s;
        uint32_t stats_total_dist_m;
        int8_t stats_min_armed_time_s;
        uint32_t stats_mah_used;
        uint8_t statsSaveMoveLimit; // gyro rate limit for saving stats upon disarm
    };
    struct stats_t {
        timeUs32_t armed_time;
        float max_g_force;
        int32_t max_altitude;
        int32_t max_esc_rpm;
        int16_t max_esc_temp_index;
        int16_t max_esc_temp;
        int16_t max_distance;
        int16_t max_speed;
        int16_t min_voltage; // /100
        uint16_t end_voltage;
        int16_t max_current; // /10
        uint16_t min_link_quality;
        int16_t min_rssi_dbm;
        int16_t min_rsnr;
        uint8_t min_rssi;
    };
    struct stats_rendering_state_t {
        uint8_t row;
        uint8_t index;
        uint8_t rowCount;
    };
public:
    void init(DisplayPortBase* displayPort, DisplayPortBase::device_type_e displayPortDeviceType);
    void drawLogoAndCompleteInitialization();
    const config_t& getConfig() const { return _config; }
    void setConfig(const config_t& config);
    const statsConfig_t& getStatsConfig() const { return _statsConfig; }
    void setStatsConfig(const statsConfig_t& statsConfig);

    OSD_Elements& getOSD_Elements() { return _elements; }
    const OSD_Elements& getOSD_Elements() const { return _elements; }

    void updateDisplay(uint32_t timeMicroseconds, uint32_t timeMicrosecondsDelta); //!< OSD Task function, called by OSD_Task
    void updateDisplayIteration(uint32_t timeMicroseconds, uint32_t timeMicrosecondsDelta);
    void drawLogo(uint8_t x, uint8_t y, DisplayPortBase::severity_e severity);

    void setStatsState(uint8_t statIndex, bool enabled);
    bool getStatsState(uint8_t statIndex) const;
    void resetStats();
    void suppressStats(bool suppressStatsFlag) { _suppressStatsFlag = suppressStatsFlag; }

    void analyzeActiveElements();

    void setWarningState(uint8_t warningIndex, bool enabled);
    bool getWarningState(uint8_t warningIndex) const;
    bool getVisualBeeperState() const { return _visualBeeperState; }
    void setVisualBeeperState(bool visualBeeperState) { _visualBeeperState = visualBeeperState; }
    const stats_t& getStats() const { return _stats; }
    void displayStatisticLabel(uint8_t x, uint8_t y, const char * text, const char * value);
    bool displayStatistic(int statistic, uint8_t displayRow);
    bool renderStatsContinue();
    bool refreshStats();
    bool processStats1(timeUs32_t currentTimeUs);
    void processStats2(timeUs32_t currentTimeUs);
    void processStats3();
    void updateAlarms();
    void syncBlink(timeUs32_t currentTimeUs);
    static int printFloat(char* buffer, char leadingSymbol, float value, char *formatString, unsigned decimalPlaces, bool round, char trailingSymbol);
private:
    DisplayPortBase* _displayPort {};
    DisplayPortBase::device_type_e _displayPortDeviceType {};
    OSD_Elements _elements;
    const Cockpit& _cockpit;
    const AHRS_MessageQueue& _ahrsMessageQueue;
    uint32_t  _resumeRefreshAtUs {};
    state_e _state { STATE_INIT };
    std::array<uint16_t, STATE_COUNT> _stateDurationFractionUs {};
    std::array<uint32_t, OSD_ITEM_COUNT> _elementDurationFractionUs {};

    config_t _config {};
    statsConfig_t _statsConfig {};
    stats_t _stats {};
    stats_rendering_state_t _statsRenderingState {};
    refresh_stats_state_e _refreshStatsState {REFRESH_STATS_STATE_INITIAL_CLEAR_SCREEN};
    bool _visualBeeperState {};
    bool _suppressStatsFlag {};
    bool _moreElementsToDraw {};
    bool _backgroundLayerSupported {};
    bool _isReady {false};
    bool _isArmed {};
    bool _statsVisible {false};
    bool _statsEnabled {false};
};
