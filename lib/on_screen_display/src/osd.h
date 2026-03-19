#pragma once

#include "osd_elements.h"

static constexpr uint8_t OSD_PROFILE_NAME_LENGTH = 16;
static constexpr uint8_t OSD_RC_CHANNELS_COUNT = 4;
static constexpr uint8_t OSD_TIMER_COUNT = 2;

struct osd_config_t {
    char profile[OSD_Elements::PROFILE_COUNT][OSD_PROFILE_NAME_LENGTH + 2]; // extra byte for zero terminator and extra byte to even-align
    int8_t rc_channels[OSD_RC_CHANNELS_COUNT];   // RC channel values to display, -1 if none
    uint16_t timers[OSD_TIMER_COUNT];

    uint32_t enabled_warnings_flags;
    uint32_t enabled_stats_flags;

    uint16_t framerate_hz;

    uint16_t cap_alarm;
    uint16_t alt_alarm;
    uint16_t link_quality_alarm;
    int16_t rssi_dbm_alarm;
    int16_t rsnr_alarm;
    uint16_t distance_alarm;
    int16_t esc_rpm_alarm;
    int16_t esc_current_alarm;
    uint8_t esc_temperature_alarm;
    uint8_t core_temperature_alarm;
    uint8_t rssi_alarm;

    uint8_t units;

    uint16_t aux_scale;
    uint8_t aux_channel;
    uint8_t aux_symbol;

    uint8_t logo_on_arming;
    uint8_t logo_on_arming_duration;        // display duration in 0.1s units
    uint8_t arming_logo_attribute;          // font attribute to use to display the logo on arming

    uint8_t ah_max_pitch;
    uint8_t ah_max_roll;
    uint8_t ah_invert;
    uint8_t osd_profile_index;
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

struct osd_stats_config_t {
    uint32_t total_flights;
    uint32_t total_time_s;
    uint32_t total_distance_m;
    uint32_t mah_used;
    int8_t min_armed_time_s;
    uint8_t save_move_limit; // gyro rate limit for saving stats upon disarm
};

struct osd_stats_t {
    time_us32_t armed_time;
    float max_g_force;
    int32_t max_altitude;
    int32_t max_esc_rpm;
    int16_t max_esc_temperature_index;
    int16_t max_esc_temperature;
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

struct osd_stats_rendering_state_t {
    uint8_t row;
    uint8_t index;
    uint8_t row_count;
};
/*!
On _screen Display.
*/
class OSD {
public:
    OSD();
private:
    // OSD is not copyable or moveable
    OSD(const OSD&) = delete;
    OSD& operator=(const OSD&) = delete;
    OSD(OSD&&) = delete;
    OSD& operator=(OSD&&) = delete;
public:
    enum { LOGO_ROW_COUNT = 4, LOGO_COLUMN_COUNT = 24 };
    enum { SD_ROWS = 16, SD_COLS = 30 };
    enum { HD_ROWS = 20, HD_COLS = 53 };
    enum { FRAMERATE_DEFAULT_HZ = 12 };
    enum { ESC_RPM_ALARM_OFF = -1, ESC_TEMPERATURE_ALARM_OFF = 0, ESC_CURRENT_ALARM_OFF = -1 };
    enum units_e { UNITS_METRIC = 0, UNITS_IMPERIAL = 1 };

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
        STATS_MIN_BATTERY,
        STATS_END_BATTERY,
        STATS_BATTERY,
        STATS_MAX_CURRENT,
        STATS_USED_MAH,
        STATS_WATT_HOURS_DRAWN,
        STATS_MAX_G_FORCE,
        STATS_MIN_RSSI,
        STATS_MIN_LINK_QUALITY,
        STATS_MIN_RSSI_DBM,
        STATS_MIN_RSNR,
#if defined(USE_BAROMETER)
        STATS_MAX_ALTITUDE,
#endif
        STATS_BLACKBOX,
        STATS_BLACKBOX_NUMBER,
#if defined(USE_DSHOT)
        STATS_MAX_ESC_TEMPERATURE,
        STATS_MAX_ESC_RPM,
#endif
        STATS_MAX_SPEED,
#if defined(USE_GPS)
        STATS_MAX_DISTANCE,
        STATS_FLIGHT_DISTANCE,
        STATS_TOTAL_DISTANCE,
#endif
        STATS_MAX_FFT,
        STATS_TOTAL_FLIGHTS,
        STATS_TOTAL_TIME,
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
        TIMER_PRECISION_SECOND,
        TIMER_PRECISION_HUNDREDTHS,
        TIMER_PRECISION_TENTHS,
        TIMER_PRECISION_COUNT
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
    void init(const DisplayPortBase* display_port);
    void draw_logo_and_complete_initialization(const osd_context_t& ctx);
    const osd_config_t& get_config() const { return _config; }
    void set_config(const osd_config_t& config);
    const osd_stats_config_t& get_stats_config() const { return _stats_config; }
    void set_stats_config(const osd_stats_config_t& stats_config);

    OSD_Elements& get_osd_elements() { return _elements; }
    const OSD_Elements& get_osd_elements() const { return _elements; }

    void set_profile(uint8_t profile) { _elements.set_profile(profile); }
    uint8_t get_profile() const { return _elements.get_profile(); }

    void update_display(const osd_context_t& ctx, uint32_t time_microseconds, uint32_t time_microseconds_delta); //!< OSD Task function, called by OSD_Task
    void update_display_iteration(const osd_context_t& ctx, uint32_t time_microseconds, uint32_t time_microseconds_delta);
    void draw_logo(DisplayPortBase& display_port, uint8_t x, uint8_t y);

    void set_stats_state(uint8_t stats_index, bool enabled);
    bool get_stats_state(uint8_t stats_index) const;
    void reset_stats();
    void suppress_stats(bool suppress_stats_flag) { _suppress_stats_flag = suppress_stats_flag; }

    void analyze_active_elements(const osd_context_t& ctx);

    void set_warning_state(uint8_t warning_index, bool enabled);
    bool get_warning_state(uint8_t warning_index) const;
    bool get_visual_beeper_state() const { return _visual_beeper_state; }
    void set_visual_beeper_state(bool visual_beeper_state) { _visual_beeper_state = visual_beeper_state; }
    const osd_stats_t& get_stats() const { return _stats; }
    void display_statistic_label(DisplayPortBase& display_port, uint8_t x, uint8_t y, const char * text, const char * value);
    bool display_statistic(DisplayPortBase& display_port, int statistic, uint8_t display_row);
    bool render_stats_continue(DisplayPortBase& display_port);
    bool refresh_stats(DisplayPortBase& display_port);
    bool process_stats1(const osd_context_t& ctx, time_us32_t current_time_us);
    void process_stats2(const osd_context_t& ctx, time_us32_t current_time_us);
    void process_stats3(const osd_context_t& ctx);
    void update_alarms();
    void sync_blink(time_us32_t current_time_us);
private:
    OSD_Elements _elements;
    uint32_t _resume_refresh_at_us {};
    state_e _state { STATE_INIT };
    std::array<uint16_t, STATE_COUNT> _state_duration_fraction_us {};
    std::array<uint32_t, OSD_ELEMENT_COUNT> _element_duration_fraction_us {};

    osd_config_t _config {};
    osd_stats_config_t _stats_config {};
    osd_stats_t _stats {};
    osd_stats_rendering_state_t _stats_rendering_state {};
    refresh_stats_state_e _refresh_stats_state {REFRESH_STATS_STATE_INITIAL_CLEAR_SCREEN};
    bool _visual_beeper_state {};
    bool _suppress_stats_flag {};
    bool _more_elements_to_draw {};
    bool _background_is_layer_supported {};
    bool _is_ready {false};
    bool _stats_visible {false};
    bool _stats_enabled {false};
#if defined(USE_OSD_STATS)
    /*!
    Controls the display order of the OSD post-flight statistics.
    Adjust the ordering here to control how the post-flight stats are presented.
    Every entry in osd_stats_e should be represented. Any that are missing will not
    be shown on the the post-flight statistics page.
    If you reorder the stats it's likely that you'll need to make likewise updates
    to the unit tests.

    If adding new stats, please add to the osdStatsNeedAccelerometer() function
    if the statistic utilizes the accelerometer.
    */
    static constexpr std::array<stats_e, STATS_COUNT> StatsDisplayOrder = {
        OSD::STATS_RTC_DATE_TIME,
        OSD::STATS_TIMER_1,
        OSD::STATS_TIMER_2,
#if defined(USE_BAROMETER)
        OSD::STATS_MAX_ALTITUDE,
#endif
        OSD::STATS_MAX_SPEED,
#if defined(USE_GPS)
        OSD::STATS_MAX_DISTANCE,
        OSD::STATS_FLIGHT_DISTANCE,
#endif
        OSD::STATS_MIN_BATTERY,
        OSD::STATS_END_BATTERY,
        OSD::STATS_BATTERY,
        OSD::STATS_MIN_RSSI,
        OSD::STATS_MAX_CURRENT,
        OSD::STATS_USED_MAH,
        OSD::STATS_BLACKBOX,
        OSD::STATS_BLACKBOX_NUMBER,
        OSD::STATS_MAX_G_FORCE,
#if defined(USE_DSHOT)
        OSD::STATS_MAX_ESC_TEMPERATURE,
        OSD::STATS_MAX_ESC_RPM,
#endif
        OSD::STATS_MIN_LINK_QUALITY,
        OSD::STATS_MAX_FFT,
        OSD::STATS_MIN_RSSI_DBM,
        OSD::STATS_MIN_RSNR,
        OSD::STATS_TOTAL_FLIGHTS,
        OSD::STATS_TOTAL_TIME,
#if defined(USE_GPS)
        OSD::STATS_TOTAL_DISTANCE,
#endif
        OSD::STATS_WATT_HOURS_DRAWN,
        OSD::STATS_BEST_3_CONSEC_LAPS,
        OSD::STATS_BEST_LAP,
        OSD::STATS_FULL_THROTTLE_TIME,
        OSD::STATS_FULL_THROTTLE_COUNTER,
        OSD::STATS_AVG_THROTTLE,
    };
#endif // USE_OSD_STATS
};
