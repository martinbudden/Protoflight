#pragma once

#include "DisplayPortBase.h"
#include "OSD_Elements.h"
#include <TimeMicroseconds.h>
#include <array>
#include <cstdint>


#ifdef USE_OSD_PROFILES
    enum { OSD_PROFILE_COUNT = 3 };
#else
    enum { OSD_PROFILE_COUNT = 1 };
#endif

enum  osd_stats_e {
    OSD_STAT_RTC_DATE_TIME,
    OSD_STAT_TIMER_1,
    OSD_STAT_TIMER_2,
    OSD_STAT_MAX_SPEED,
    OSD_STAT_MAX_DISTANCE,
    OSD_STAT_MIN_BATTERY,
    OSD_STAT_END_BATTERY,
    OSD_STAT_BATTERY,
    OSD_STAT_MIN_RSSI,
    OSD_STAT_MAX_CURRENT,
    OSD_STAT_USED_MAH,
    OSD_STAT_MAX_ALTITUDE,
    OSD_STAT_BLACKBOX,
    OSD_STAT_BLACKBOX_NUMBER,
    OSD_STAT_MAX_G_FORCE,
    OSD_STAT_MAX_ESC_TEMP,
    OSD_STAT_MAX_ESC_RPM,
    OSD_STAT_MIN_LINK_QUALITY,
    OSD_STAT_FLIGHT_DISTANCE,
    OSD_STAT_MAX_FFT,
    OSD_STAT_TOTAL_FLIGHTS,
    OSD_STAT_TOTAL_TIME,
    OSD_STAT_TOTAL_DIST,
    OSD_STAT_MIN_RSSI_DBM,
    OSD_STAT_WATT_HOURS_DRAWN,
    OSD_STAT_MIN_RSNR,
    OSD_STAT_BEST_3_CONSEC_LAPS,
    OSD_STAT_BEST_LAP,
    OSD_STAT_FULL_THROTTLE_TIME,
    OSD_STAT_FULL_THROTTLE_COUNTER,
    OSD_STAT_AVG_THROTTLE,
    OSD_STAT_COUNT // MUST BE LAST
};


enum osd_timer_e {
    OSD_TIMER_1,
    OSD_TIMER_2,
    OSD_TIMER_COUNT
};

enum osd_timer_source_e {
    OSD_TIMER_SRC_ON,
    OSD_TIMER_SRC_TOTAL_ARMED,
    OSD_TIMER_SRC_LAST_ARMED,
    OSD_TIMER_SRC_ON_OR_ARMED,
    OSD_TIMER_SRC_LAUNCH_TIME,
    OSD_TIMER_SRC_COUNT
};

enum osd_timer_precision_e {
    OSD_TIMER_PREC_SECOND,
    OSD_TIMER_PREC_HUNDREDTHS,
    OSD_TIMER_PREC_TENTHS,
    OSD_TIMER_PREC_COUNT
};

enum osd_warnings_flags_e {
    OSD_WARNING_ARMING_DISABLE,
    OSD_WARNING_BATTERY_NOT_FULL,
    OSD_WARNING_BATTERY_WARNING,
    OSD_WARNING_BATTERY_CRITICAL,
    OSD_WARNING_VISUAL_BEEPER,
    OSD_WARNING_CRASHFLIP,
    OSD_WARNING_ESC_FAIL,
    OSD_WARNING_CORE_TEMPERATURE,
    OSD_WARNING_RC_SMOOTHING,
    OSD_WARNING_FAIL_SAFE,
    OSD_WARNING_LAUNCH_CONTROL,
    OSD_WARNING_GPS_RESCUE_UNAVAILABLE,
    OSD_WARNING_GPS_RESCUE_DISABLED,
    OSD_WARNING_RSSI,
    OSD_WARNING_LINK_QUALITY,
    OSD_WARNING_RSSI_DBM,
    OSD_WARNING_OVER_CAP,
    OSD_WARNING_RSNR,
    OSD_WARNING_LOAD,
    OSD_WARNING_POSHOLD_FAILED,
    OSD_WARNING_COUNT // MUST BE LAST
};

class OSD {
public:
    OSD(const FlightController& flightController);
private:
    // OSD is not copyable or moveable
    OSD(const OSD&) = delete;
    OSD& operator=(const OSD&) = delete;
    OSD(OSD&&) = delete;
    OSD& operator=(OSD&&) = delete;
public:
    enum { ELEMENT_BUFFER_LENGTH = 32 };
    enum { PROFILE_NAME_LENGTH = 16 };
    enum { RC_CHANNELS_COUNT = 4 };
    enum { OSD_LOGO_ROW_COUNT = 4 };
    enum { OSD_LOGO_COLUMN_COUNT = 24 };
    enum  state_e {
        STATE_INIT,
        STATE_IDLE,
        STATE_CHECK,
        STATE_PROCESS_STATS1,
        STATE_REFRESH_STATS,
        STATE_PROCESS_STATS2,
        STATE_PROCESS_STATS3,
        STATE_UPDATE_ALARMS,
        STATE_REFRESH_PREARM,
        STATE_UPDATE_CANVAS,
        // Elements are handled in two steps, drawing into a buffer, and then sending to the display
        STATE_DRAW_ELEMENT,
        STATE_DISPLAY_ELEMENT,
        STATE_UPDATE_HEARTBEAT,
        STATE_COMMIT,
        STATE_TRANSFER,
        STATE_COUNT
    };
public:
    struct config_t {
        char profile[OSD_PROFILE_COUNT][PROFILE_NAME_LENGTH + 2];
        int8_t rcChannels[RC_CHANNELS_COUNT];   // RC channel values to display, -1 if none
        uint16_t timers[OSD_TIMER_COUNT];
        uint32_t enabledWarnings;
        uint32_t enabled_stats;
        uint16_t cap_alarm;
        uint16_t alt_alarm;
        uint16_t link_quality_alarm;
        int16_t rssi_dbm_alarm;
        int16_t rsnr_alarm;
        uint16_t distance_alarm;
        uint16_t framerate_hz;
        int16_t esc_rpm_alarm;
        int16_t esc_current_alarm;
        uint16_t aux_scale;
        uint8_t aux_channel;
        uint8_t aux_symbol;
        uint8_t rssi_alarm;
        uint8_t units;
        uint8_t ahMaxPitch;
        uint8_t ahMaxRoll;
        uint8_t esc_temp_alarm;
        uint8_t core_temp_alarm;
        uint8_t artificial_horizon_invert;
        uint8_t osdProfileIndex;
        uint8_t overlay_radio_mode;
        uint8_t gps_sats_show_pdop;
        uint8_t displayPortDevice;
        uint8_t logo_on_arming;
        uint8_t logo_on_arming_duration;        // display duration in 0.1s units
        uint8_t camera_frame_width;
        uint8_t camera_frame_height;
        uint8_t cms_background_type;            // whether the CMS background is transparent or opaque
        uint8_t stat_show_cell_value;
        uint8_t osd_craftname_messages;         // Insert LQ/RSSI-dBm and warnings into CraftName
        uint8_t canvas_column_count;
        uint8_t canvas_row_count;
        uint8_t osd_use_quick_menu;
        uint8_t osd_show_spec_prearm;
        DisplayPortBase::severity_e arming_logo; // font from which to display the logo on arming
    };
    struct statistics_t {
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
    struct statistics_rendering_state_t {
        uint8_t row;
        uint8_t index;
        uint8_t rowCount;
    };
public:
    void init(DisplayPortBase *displayPort, DisplayPortBase::device_type_e displayPortDeviceType);
    void completeInitialization();
    const config_t& getConfig() const { return _config; }
    void setConfig(const config_t& config);

    bool updateOSD(uint32_t timeMicroseconds, uint32_t timeMicrosecondsDelta);
    void drawLogo(uint8_t x, uint8_t y, DisplayPortBase::severity_e severity);

    void setStatisticsState(uint8_t statIndex, bool enabled);
    bool getStatisticsState(uint8_t statIndex) const;
    void resetStatistics();
    void suppressStatistics(bool suppressStatsFlag) { _suppressStatsFlag = suppressStatsFlag; }

    void analyzeActiveElements();

    void setWarningState(uint8_t warningIndex, bool enabled);
    bool getWarningState(uint8_t warningIndex) const;
    bool elementVisible(uint16_t value) const;
    bool getVisualBeeperState() const { return _visualBeeperState; }
    void setVisualBeeperState(bool visualBeeperState) { _visualBeeperState = visualBeeperState; }
    const statistics_t& getStatistics() const { return _statistics; }

    static int printFloat(char *buffer, char leadingSymbol, float value, char *formatString, unsigned decimalPlaces, bool round, char trailingSymbol);
private:
    DisplayPortBase* _displayPort {};
    DisplayPortBase::device_type_e _displayPortDeviceType {};
    OSD_Elements _elements;
    state_e _state { STATE_INIT };
    std::array<uint16_t, STATE_COUNT> _stateDurationFractionUs {};
    std::array<uint32_t, OSD_ITEM_COUNT> _elementDurationFractionUs {};

    config_t _config {};
    statistics_t _statistics {};
    statistics_rendering_state_t _statisticRenderingState {};
    bool _visualBeeperState {};
    bool _suppressStatsFlag {};
    bool _moreElementsToDraw {};
};
