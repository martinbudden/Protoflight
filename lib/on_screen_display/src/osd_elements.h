#pragma once

#include "targets.h"

#include <array>
#include <bitset>
#include <cstdint>
#include <time_microseconds.h>

class DisplayPortBase;
class OSD;
class AhrsMessageQueue;
class Cockpit;
class Debug;
class FlightController;
class GPS;
class OSD;
class RcModes;
class ReceiverBase;
class VTX;

struct osd_context_t {
    DisplayPortBase& display_port;
    const AhrsMessageQueue& ahrs_message_queue;
    const FlightController& flight_controller;
    const Cockpit& cockpit;
    const ReceiverBase& receiver;
    const RcModes& rc_modes;
    const Debug& debug;
    const VTX* vtx;
    const GPS* gps;
};


enum  osd_elements_e {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATTERY_VOLTAGE,
    OSD_CROSSHAIRS,
    OSD_ARTIFICIAL_HORIZON,
    OSD_HORIZON_SIDEBARS,
    OSD_ITEM_TIMER_1,
    OSD_ITEM_TIMER_2,
    OSD_FLYMODE,
    OSD_CRAFT_NAME,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
#if defined(USE_GPS)
    OSD_GPS_SPEED,
    OSD_GPS_SATS,
#endif
#if defined(USE_BAROMETER) || defined(USE_GPS)
    OSD_ALTITUDE,
#endif
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_PID_RATE_PROFILE,
    OSD_WARNINGS,
    OSD_AVG_CELL_VOLTAGE,
#if defined(USE_GPS)
    OSD_GPS_LON,
    OSD_GPS_LAT,
#endif
    OSD_DEBUG,
    OSD_PITCH_ANGLE,
    OSD_ROLL_ANGLE,
    OSD_MAIN_BATTERY_USAGE,
    OSD_DISARMED,
#if defined(USE_GPS)
    OSD_HOME_DIRECTION,
    OSD_HOME_DISTANCE,
#endif
    OSD_NUMERICAL_HEADING,
    OSD_NUMERICAL_VARIO,
    OSD_COMPASS_BAR,
#if defined(USE_DSHOT)
    OSD_ESC_TEMPERATURE,
    OSD_ESC_RPM,
#endif
    OSD_REMAINING_TIME_ESTIMATE,
#if defined(USE_RTC_TIME)
    OSD_RTC_DATETIME,
#endif
#if defined(USE_OSD_ADJUSTMENTS)
    OSD_ADJUSTMENT_RANGE,
#endif
    OSD_CORE_TEMPERATURE,
    OSD_ANTI_GRAVITY,
    OSD_G_FORCE,
#if defined(USE_DSHOT)
    OSD_MOTOR_DIAGNOSTICS,
#endif
    OSD_LOG_STATUS,
    OSD_FLIP_ARROW,
    OSD_LINK_QUALITY,
#if defined(USE_GPS)
    OSD_FLIGHT_DISTANCE,
#endif
    OSD_STICK_OVERLAY_LEFT,
    OSD_STICK_OVERLAY_RIGHT,
#if defined(USE_OSD_PROFILES)
    OSD_PILOT_NAME,
#endif
#if defined(USE_DSHOT)
    OSD_ESC_RPM_FREQUENCY,
#endif
#if defined(USE_PROFILE_NAMES)
    OSD_RATE_PROFILE_NAME,
    OSD_PID_PROFILE_NAME,
#endif
#if defined(USE_OSD_PROFILES)
    OSD_PROFILE_NAME,
#endif
    OSD_RSSI_DBM_VALUE,
    OSD_RC_CHANNELS,
    OSD_CAMERA_FRAME,
#if defined(USE_GPS)
    OSD_EFFICIENCY,
#endif
    OSD_TOTAL_FLIGHTS,
    OSD_UP_DOWN_REFERENCE,
    OSD_TX_UPLINK_POWER,
    OSD_WATT_HOURS_DRAWN,
    OSD_AUX_VALUE,
    OSD_READY_MODE,
    OSD_RSNR_VALUE,
#if defined(USE_OSD_HD)
    OSD_SYS_GOGGLE_VOLTAGE,
    OSD_SYS_VTX_VOLTAGE,
    OSD_SYS_BITRATE,
    OSD_SYS_DELAY,
    OSD_SYS_DISTANCE,
    OSD_SYS_LQ,
    OSD_SYS_GOGGLE_DVR,
    OSD_SYS_VTX_DVR,
    OSD_SYS_WARNINGS,
    OSD_SYS_VTX_TEMPERATURE,
    OSD_SYS_FAN_SPEED,
#endif
#if defined(USE_GPS_LAP_TIMER)
    OSD_GPS_LAP_TIME_CURRENT,
    OSD_GPS_LAP_TIME_PREVIOUS,
    OSD_GPS_LAP_TIME_BEST3,
#endif
    OSD_DEBUG2,
#if defined(USE_OSD_PROFILES)
    OSD_CUSTOM_MSG0,
    OSD_CUSTOM_MSG1,
    OSD_CUSTOM_MSG2,
    OSD_CUSTOM_MSG3,
#endif
#if defined(USE_RANGEFINDER)
    OSD_LIDAR_DISTANCE,
#endif
    OSD_ELEMENT_COUNT // MUST BE LAST
};

struct osd_elements_config_t {
    std::array<uint16_t, OSD_ELEMENT_COUNT> element_pos; // 2 bits for type, 2 bits for profile, 6 bits for y, 6 bits for x
};

/*!
How to add a new OSD element:

1. Create a new enum, say, `OSD_MY_ELEMENT`, and add it to the `osd_elements_e` enumeration list above.
2. Create a drawing function `void OSD::Elements::draw_MY_ELEMENT(DisplayPortBase& display_port)`
   and optionally a background drawing function `void OSD::Elements::drawBackground_MY_ELEMENT(DisplayPortBase& display_port)`.
3. Add the drawing function to the `DrawFunctions` array in `OSD_Elements::initDrawFunctions()`.
4. If you created a background drawing function then add it to the `DrawBackgroundFunctions` array.
5. Add `OSD_MY_ELEMENT` to the active elements in `OSD_Elements::add_active_elements()`.
*/
class OSD_Elements {
public:
    OSD_Elements(const OSD& osd);
    void init(bool backgroundLayerFlag, uint8_t row_count, uint8_t column_count);
    void initDrawFunctions();
public:
    enum element_type_e{
        OSD_ELEMENT_TYPE_1 = 0,
        OSD_ELEMENT_TYPE_2,
        OSD_ELEMENT_TYPE_3,
        OSD_ELEMENT_TYPE_4
    };
    enum { STICK_OVERLAY_WIDTH = 7, STICK_OVERLAY_HEIGHT = 5 };
    struct element_t {
        enum { ELEMENT_BUFFER_LENGTH = 32 };
        std::array<char, ELEMENT_BUFFER_LENGTH> buf;
        element_type_e type;
        uint8_t index;
        uint8_t pos_x;
        uint8_t pos_y;
        uint8_t offset_x;
        uint8_t offset_y;
        uint8_t attr;
        bool rendered;
        bool draw_element;
    };
    typedef void (OSD_Elements::*elementDrawFnPtr)(const osd_context_t& ctx);

public:
#if defined(USE_OSD_HD)
    bool isSysOSD_Element(uint8_t index) { return (index >= OSD_SYS_GOGGLE_VOLTAGE) && (index <= OSD_SYS_FAN_SPEED); }
#else
    bool isSysOSD_Element(uint8_t index) { (void)index; return false; }
#endif

    const osd_elements_config_t& get_config() const { return _config; }
    osd_elements_config_t& get_config() { return _config; }
    void set_config(const osd_elements_config_t& config);
    void set_default_config(uint8_t row_count, uint8_t column_count);

    static constexpr uint16_t ELEMENT_BITS_POS = 14;
    static constexpr uint32_t ELEMENT_TYPE_MASK = 0b1100'0000'0000'0000U;  // bits 14-15
    static element_type_e ELEMENT_TYPE(uint32_t x) { return static_cast<element_type_e>((x & ELEMENT_TYPE_MASK) >> ELEMENT_BITS_POS); }

#if defined(USE_OSD_PROFILES)
    enum { PROFILE_COUNT = 2 };
#else
    enum { PROFILE_COUNT = 2 };
#endif
    static constexpr uint16_t PROFILE_BITS_POS = 12;
    static constexpr uint16_t PROFILE_MASK = 0b0011'0000'0000'0000U;
    static uint16_t profile_flag(uint16_t x) { return 1U << (x - 1 + PROFILE_BITS_POS); }
    static bool element_visible(uint16_t value, uint16_t profile) { return ((value & PROFILE_MASK) >> PROFILE_BITS_POS) & (1 << profile); }

    static constexpr uint16_t XY_POSITION_BITS = 6;       // 6 bits gives a range 0-63
    static constexpr uint16_t XY_POSITION_MASK = 0b11'1111U;

    static uint8_t OSD_X(uint16_t x) { return x & XY_POSITION_MASK; }
    static uint8_t OSD_Y(uint16_t x) { return (x >> XY_POSITION_BITS) & XY_POSITION_MASK; }
    static uint16_t OSD_POS(uint8_t x, uint8_t y) { return (x & XY_POSITION_MASK) | ((y & XY_POSITION_MASK) << XY_POSITION_BITS); }

    void set_profile(uint8_t profile);
    uint8_t get_profile() const { return _profile; }

    void add_active_element(osd_elements_e element);
    void add_active_elements(const osd_context_t& ctx);
    bool isRenderPending() const;
    uint8_t get_active_element_index() const { return _active_element_index; }
    uint8_t get_active_elementCount() const { return _active_element_count; }

    void update_attitude(float roll_angle_degrees, float pitch_angle_degrees, float yaw_angle_degrees);
    bool draw_next_active_element(const osd_context_t& ctx);
    bool display_active_element(DisplayPortBase& display_port);
    void draw_active_elements_background(const osd_context_t& ctx);

    bool draw_spec(const osd_context_t& ctx);

    bool draw_single_element(const osd_context_t& ctx, uint8_t element_index);
    bool draw_single_element_background(const osd_context_t& ctx, uint8_t element_index);

    uint32_t display_write_string(DisplayPortBase& display_port, const element_t& element, uint8_t x, uint8_t y, const char* s, uint8_t attr);
    uint32_t display_write_char(DisplayPortBase& display_port, const element_t& element, uint8_t x, uint8_t y, uint8_t c, uint8_t attr);

// element drawing functions
    void format_distance_string(char* buf, float distance, char leadingSymbol);
    static int printFloat(char* buffer, char leadingSymbol, float value, unsigned decimal_places, bool round, char trailingSymbol);
    void formatPID(const osd_context_t& ctx, char* buf, const char* label, uint8_t axis);
    // no need to qualify functions with #if defined(USE_...) since linker will remove unused functions
    void draw_RSSI_VALUE(const osd_context_t& ctx);
    void draw_MAIN_BATTERY_VOLTAGE(const osd_context_t& ctx);
    void draw_CROSSHAIRS(const osd_context_t& ctx); // only has background, but needs to be over other elements (like artificial horizon)
    void draw_ARTIFICIAL_HORIZON(const osd_context_t& ctx);
    void draw_ITEM_TIMER(const osd_context_t& ctx);
    void draw_FLYMODE(const osd_context_t& ctx);
    void draw_THROTTLE_POS(const osd_context_t& ctx);
    void draw_VTX_CHANNEL(const osd_context_t& ctx);
    void draw_CURRENT_DRAW(const osd_context_t& ctx);
    void draw_MAH_DRAWN(const osd_context_t& ctx);
    void draw_GPS_SPEED(const osd_context_t& ctx);
    void draw_GPS_SATS(const osd_context_t& ctx);
    void draw_ALTITUDE(const osd_context_t& ctx);
    void draw_ROLL_PIDS(const osd_context_t& ctx);
    void draw_PITCH_PIDS(const osd_context_t& ctx);
    void draw_YAW_PIDS(const osd_context_t& ctx);
    void draw_POWER(const osd_context_t& ctx);
    void draw_PID_RATE_PROFILE(const osd_context_t& ctx);
    void draw_WARNINGS(const osd_context_t& ctx);
    void draw_AVG_CELL_VOLTAGE(const osd_context_t& ctx);
    void draw_GPS_LAT_LONG(const osd_context_t& ctx);
    void draw_DEBUG(const osd_context_t& ctx);
    void draw_PITCH_ANGLE(const osd_context_t& ctx);
    void draw_ROLL_ANGLE(const osd_context_t& ctx);
    void draw_MAIN_BATTERY_USAGE(const osd_context_t& ctx);
    void draw_DISARMED(const osd_context_t& ctx);
    void draw_HOME_DIRECTION(const osd_context_t& ctx);
    void draw_HOME_DISTANCE(const osd_context_t& ctx);
    void draw_NUMERICAL_HEADING(const osd_context_t& ctx);
    void draw_NUMERICAL_VARIO(const osd_context_t& ctx);
    void draw_COMPASS_BAR(const osd_context_t& ctx);
    void draw_ESC_TEMPERATURE(const osd_context_t& ctx);
    void draw_ESC_RPM(const osd_context_t& ctx);
    void draw_REMAINING_TIME_ESTIMATE(const osd_context_t& ctx);
    void draw_RTC_DATETIME(const osd_context_t& ctx);
    void draw_ADJUSTMENT_RANGE(const osd_context_t& ctx);
    void draw_CORE_TEMPERATURE(const osd_context_t& ctx);
    void draw_ANTI_GRAVITY(const osd_context_t& ctx);
    void draw_G_FORCE(const osd_context_t& ctx);
    void draw_MOTOR_DIAGNOSTICS(const osd_context_t& ctx);
    void draw_LOG_STATUS(const osd_context_t& ctx);
    void draw_FLIP_ARROW(const osd_context_t& ctx);
    void draw_LINK_QUALITY(const osd_context_t& ctx);
    void draw_FLIGHT_DISTANCE(const osd_context_t& ctx);
    void draw_STICK_OVERLAY(const osd_context_t& ctx);
    void draw_PILOT_NAME(const osd_context_t& ctx);
    void draw_ESC_RPM_FREQUENCY(const osd_context_t& ctx);
    void draw_RATE_PROFILE_NAME(const osd_context_t& ctx);
    void draw_PID_PROFILE_NAME(const osd_context_t& ctx);
    void draw_PROFILE_NAME(const osd_context_t& ctx);
    void draw_RSSI_DBM_VALUE(const osd_context_t& ctx);
    void draw_RC_CHANNELS(const osd_context_t& ctx);
    void draw_EFFICIENCY(const osd_context_t& ctx);
    void draw_TOTAL_FLIGHTS(const osd_context_t& ctx);
    void draw_UP_DOWN_REFERENCE(const osd_context_t& ctx);
    void draw_TX_UPLINK_POWER(const osd_context_t& ctx);
    void draw_WATT_HOURS_DRAWN(const osd_context_t& ctx);
    void draw_AUX_VALUE(const osd_context_t& ctx);
    void draw_READY_MODE(const osd_context_t& ctx);
    void draw_RSNR_VALUE(const osd_context_t& ctx);
    void draw_SYS(const osd_context_t& ctx);
    void draw_GPS_LAP_TIME_CURRENT(const osd_context_t& ctx);
    void draw_GPS_LAP_TIME_PREVIOUS(const osd_context_t& ctx);
    void draw_GPS_LAP_TIME_BEST3(const osd_context_t& ctx);
    void draw_DEBUG2(const osd_context_t& ctx);
    void draw_CUSTOM_MSG(const osd_context_t& ctx);
    void draw_LIDAR_DISTANCE(const osd_context_t& ctx);
// element background drawing functions
    void drawBackground_HORIZON_SIDEBARS(const osd_context_t& ctx);
    void drawBackground_CRAFT_NAME(const osd_context_t& ctx);
    void drawBackground_STICK_OVERLAY(const osd_context_t& ctx);
    void drawBackground_PILOT_NAME(const osd_context_t& ctx);
    void drawBackground_CAMERA_FRAME(const osd_context_t& ctx);
private:
    const OSD& _osd;
    float _roll_angle_degrees {};
    float _pitch_angle_degrees {};
    float _yaw_angle_degrees {};
    element_t _active_element {};
    uint8_t _active_element_index = 0;
    uint8_t _active_element_count = 0;
    uint8_t _profile {};
    bool _displayPendingForeground {};
    bool _displayPendingBackground {};
    //bool _blinkState {true};
    bool _backgroundRendered {false};
    bool _background_layer_supported {false};

    // data 'local' to specific draw functions
    bool _HORIZON_SIDEBARS_RenderLevel {false};
    enum { AH_SIDEBAR_WIDTH_POS = 7, AH_SIDEBAR_HEIGHT_POS = 3 };
    int8_t _HORIZON_SIDEBARS__pos_y {AH_SIDEBAR_HEIGHT_POS};
    uint8_t _RC_CHANNELS_channel {};
    enum {VERTICAL, HORIZONTAL} _STICK_OVERLAY_RenderPhase {VERTICAL};
    uint8_t _STICK_OVERLAY_Y {0};

    osd_elements_config_t _config {};
    std::array<uint8_t, OSD_ELEMENT_COUNT> _active_elements;
    std::bitset<OSD_ELEMENT_COUNT> _blink_bits {};
private:
    // drawing functions
    static std::array<OSD_Elements::elementDrawFnPtr, OSD_ELEMENT_COUNT> DrawFunctions;
    static std::array<OSD_Elements::elementDrawFnPtr, OSD_ELEMENT_COUNT> DrawBackgroundFunctions;
};
