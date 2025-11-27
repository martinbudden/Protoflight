#pragma once

#include <TimeMicroseconds.h>
#include <array>
#include <bitset>
#include <cstdint>

class Cockpit;
class Debug;
class DisplayPortBase;
class FlightController;
class OSD;

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
#if defined(USE_DSHOT_TELEMETRY) || defined(USE_ESC_SENSOR)
    OSD_ESC_TEMPERATURE,
    OSD_ESC_RPM,
#endif
    OSD_REMAINING_TIME_ESTIMATE,
    OSD_RTC_DATETIME,
    OSD_ADJUSTMENT_RANGE,
    OSD_CORE_TEMPERATURE,
    OSD_ANTI_GRAVITY,
    OSD_G_FORCE,
    OSD_MOTOR_DIAGNOSTICS,
    OSD_LOG_STATUS,
    OSD_FLIP_ARROW,
    OSD_LINK_QUALITY,
#if defined(USE_GPS)
    OSD_FLIGHT_DISTANCE,
#endif
    OSD_STICK_OVERLAY_LEFT,
    OSD_STICK_OVERLAY_RIGHT,
    OSD_PILOT_NAME,
#if defined(USE_DSHOT_TELEMETRY) || defined(USE_ESC_SENSOR)
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
    OSD_CUSTOM_MSG0,
    OSD_CUSTOM_MSG1,
    OSD_CUSTOM_MSG2,
    OSD_CUSTOM_MSG3,
#if defined(USE_RANGEFINDER)
    OSD_LIDAR_DISTANCE,
#endif
    OSD_ITEM_COUNT // MUST BE LAST
};

/*!
How to add a new OSD element:

1. Create a new enum, say, `OSD_MY_ELEMENT`, and add it to the `osd_elements_e` enumeration list above.
2. Create a drawing function `void OSD::Elements::draw_MY_ELEMENT(DisplayPortBase& displayPort)`
   and optionally a background drawing function `void OSD::Elements::drawBackground_MY_ELEMENT(DisplayPortBase& displayPort)`.
3. Add the drawing function to the `DrawFunctions` array in `OSD_Elements::initDrawFunctions()`.
4. If you created a background drawing function then add it to the `DrawBackgroundFunctions` array.
5. Add `OSD_MY_ELEMENT` to the active elements in `OSD_Elements::addActiveElements()`.
*/
class OSD_Elements {
public:
    OSD_Elements(const OSD& osd, const FlightController& flightController, const Cockpit& cockpit, const Debug& debug);
    void init(bool backgroundLayerFlag, uint8_t rowCount, uint8_t columnCount);
    void initDrawFunctions();
public:
    enum element_type_e{
        OSD_ELEMENT_TYPE_1 = 0,
        OSD_ELEMENT_TYPE_2,
        OSD_ELEMENT_TYPE_3,
        OSD_ELEMENT_TYPE_4
    };
    enum { STICK_OVERLAY_WIDTH = 7, STICK_OVERLAY_HEIGHT = 5 };
    struct config_t {
        std::array<uint16_t, OSD_ITEM_COUNT> element_pos; // 2 bits for type, 2 bits for profile, 6 bits for y, 6 bits for x
    };
    struct element_t {
        enum { ELEMENT_BUFFER_LENGTH = 32 };
        std::array<char, ELEMENT_BUFFER_LENGTH> buf;
        element_type_e type;
        uint8_t index;
        uint8_t posX;
        uint8_t posY;
        uint8_t offsetX;
        uint8_t offsetY;
        uint8_t attr;
        bool rendered;
        bool drawElement;
    };
    typedef void (OSD_Elements::*elementDrawFnPtr)(DisplayPortBase& displayPort);

public:
#if defined(USE_OSD_HD)
    bool isSysOSD_Element(uint8_t index) { return (index >= OSD_SYS_GOGGLE_VOLTAGE) && (index <= OSD_SYS_FAN_SPEED); }
#else
    bool isSysOSD_Element(uint8_t index) { (void)index; return false; }
#endif

    const config_t& getConfig() const { return _config; }
    config_t& getConfig() { return _config; }
    void setConfig(const config_t& config);
    void setDefaultConfig(uint8_t rowCount, uint8_t columnCount);

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
    static uint16_t profileFlag(uint16_t x) { return 1U << (x - 1 + PROFILE_BITS_POS); }
    static bool elementVisible(uint16_t value, uint16_t profile) { return ((value & PROFILE_MASK) >> PROFILE_BITS_POS) & (1 << profile); }

    static constexpr uint16_t XY_POSITION_BITS = 6;       // 6 bits gives a range 0-63
    static constexpr uint16_t XY_POSITION_MASK = 0b11'1111U;

    static uint8_t OSD_X(uint16_t x) { return x & XY_POSITION_MASK; }
    static uint8_t OSD_Y(uint16_t x) { return (x >> XY_POSITION_BITS) & XY_POSITION_MASK; }
    static uint16_t OSD_POS(uint8_t x, uint8_t y) { return (x & XY_POSITION_MASK) | ((y & XY_POSITION_MASK) << XY_POSITION_BITS); }

    void addActiveElement(osd_elements_e element);
    void addActiveElements();
    bool isRenderPending() const;
    uint8_t getActiveElementIndex() const { return _activeElementIndex; }
    uint8_t getActiveElementCount() const { return _activeElementCount; }

    void updateAttitude(float rollAngleDegrees, float pitchAngleDegrees, float yawAngleDegrees);
    bool drawNextActiveElement(DisplayPortBase& displayPort);
    bool displayActiveElement(DisplayPortBase& displayPort);
    void drawActiveElementsBackground(DisplayPortBase& displayPort);

    bool drawSpec(DisplayPortBase& displayPort);

    bool drawSingleElement(DisplayPortBase& displayPort, uint8_t elementIndex);
    bool drawSingleElementBackground(DisplayPortBase& displayPort, uint8_t elementIndex);

    uint32_t displayWrite(DisplayPortBase& displayPort, const element_t& element, uint8_t x, uint8_t y, uint8_t attr, const char* s);
    uint32_t displayWrite(DisplayPortBase& displayPort, const element_t& element, uint8_t x, uint8_t y, uint8_t attr, uint8_t c);

// element drawing functions
    void formatPID(char* buf, const char* label, uint8_t axis);
    void draw_RSSI_VALUE(DisplayPortBase& displayPort);
    void draw_MAIN_BATTERY_VOLTAGE(DisplayPortBase& displayPort);
    void draw_CROSSHAIRS(DisplayPortBase& displayPort); // only has background, but needs to be over other elements (like artificial horizon)
    void draw_ARTIFICIAL_HORIZON(DisplayPortBase& displayPort);
    void draw_ITEM_TIMER(DisplayPortBase& displayPort);
    void draw_FLYMODE(DisplayPortBase& displayPort);
    void draw_THROTTLE_POS(DisplayPortBase& displayPort);
    void draw_VTX_CHANNEL(DisplayPortBase& displayPort);
    void draw_CURRENT_DRAW(DisplayPortBase& displayPort);
    void draw_MAH_DRAWN(DisplayPortBase& displayPort);
    void draw_GPS_SPEED(DisplayPortBase& displayPort);
    void draw_GPS_SATS(DisplayPortBase& displayPort);
    void draw_ALTITUDE(DisplayPortBase& displayPort);
    void draw_ROLL_PIDS(DisplayPortBase& displayPort);
    void draw_PITCH_PIDS(DisplayPortBase& displayPort);
    void draw_YAW_PIDS(DisplayPortBase& displayPort);
    void draw_POWER(DisplayPortBase& displayPort);
    void draw_PID_RATE_PROFILE(DisplayPortBase& displayPort);
    void draw_WARNINGS(DisplayPortBase& displayPort);
    void draw_AVG_CELL_VOLTAGE(DisplayPortBase& displayPort);
    void draw_GPS_LAT_LONG(DisplayPortBase& displayPort);
    void draw_DEBUG(DisplayPortBase& displayPort);
    void draw_PITCH_ANGLE(DisplayPortBase& displayPort);
    void draw_ROLL_ANGLE(DisplayPortBase& displayPort);
    void draw_MAIN_BATTERY_USAGE(DisplayPortBase& displayPort);
    void draw_DISARMED(DisplayPortBase& displayPort);
    void draw_HOME_DIRECTION(DisplayPortBase& displayPort);
    void draw_HOME_DISTANCE(DisplayPortBase& displayPort);
    void draw_NUMERICAL_HEADING(DisplayPortBase& displayPort);
    void draw_NUMERICAL_VARIO(DisplayPortBase& displayPort);
    void draw_COMPASS_BAR(DisplayPortBase& displayPort);
    void draw_ESC_TEMPERATURE(DisplayPortBase& displayPort);
    void draw_ESC_RPM(DisplayPortBase& displayPort);
    void draw_REMAINING_TIME_ESTIMATE(DisplayPortBase& displayPort);
    void draw_RTC_DATETIME(DisplayPortBase& displayPort);
    void draw_ADJUSTMENT_RANGE(DisplayPortBase& displayPort);
    void draw_CORE_TEMPERATURE(DisplayPortBase& displayPort);
    void draw_ANTI_GRAVITY(DisplayPortBase& displayPort);
    void draw_G_FORCE(DisplayPortBase& displayPort);
    void draw_MOTOR_DIAGNOSTICS(DisplayPortBase& displayPort);
    void draw_LOG_STATUS(DisplayPortBase& displayPort);
    void draw_FLIP_ARROW(DisplayPortBase& displayPort);
    void draw_LINK_QUALITY(DisplayPortBase& displayPort);
    void draw_FLIGHT_DISTANCE(DisplayPortBase& displayPort);
    void draw_STICK_OVERLAY(DisplayPortBase& displayPort);
    void draw_PILOT_NAME(DisplayPortBase& displayPort);
    void draw_ESC_RPM_FREQUENCY(DisplayPortBase& displayPort);
    void draw_RATE_PROFILE_NAME(DisplayPortBase& displayPort);
    void draw_PID_PROFILE_NAME(DisplayPortBase& displayPort);
    void draw_PROFILE_NAME(DisplayPortBase& displayPort);
    void draw_RSSI_DBM_VALUE(DisplayPortBase& displayPort);
    void draw_RC_CHANNELS(DisplayPortBase& displayPort);
    void draw_EFFICIENCY(DisplayPortBase& displayPort);
    void draw_TOTAL_FLIGHTS(DisplayPortBase& displayPort);
    void draw_UP_DOWN_REFERENCE(DisplayPortBase& displayPort);
    void draw_TX_UPLINK_POWER(DisplayPortBase& displayPort);
    void draw_WATT_HOURS_DRAWN(DisplayPortBase& displayPort);
    void draw_AUX_VALUE(DisplayPortBase& displayPort);
    void draw_READY_MODE(DisplayPortBase& displayPort);
    void draw_RSNR_VALUE(DisplayPortBase& displayPort);
    void draw_SYS(DisplayPortBase& displayPort);
    void draw_GPS_LAP_TIME_CURRENT(DisplayPortBase& displayPort);
    void draw_GPS_LAP_TIME_PREVIOUS(DisplayPortBase& displayPort);
    void draw_GPS_LAP_TIME_BEST3(DisplayPortBase& displayPort);
    void draw_DEBUG2(DisplayPortBase& displayPort);
    void draw_CUSTOM_MSG(DisplayPortBase& displayPort);
    void draw_LIDAR_DISTANCE(DisplayPortBase& displayPort);
// element background drawing functions
    void drawBackground_HORIZON_SIDEBARS(DisplayPortBase& displayPort);
    void drawBackground_CRAFT_NAME(DisplayPortBase& displayPort);
    void drawBackground_STICK_OVERLAY(DisplayPortBase& displayPort);
    void drawBackground_PILOT_NAME(DisplayPortBase& displayPort);
    void drawBackground_CAMERA_FRAME(DisplayPortBase& displayPort);
private:
    const OSD& _osd;
    const FlightController& _flightController;
    const Cockpit& _cockpit;
    const Debug& _debug;
    float _rollAngleDegrees {};
    float _pitchAngleDegrees {};
    float _yawAngleDegrees {};
    element_t _activeElement {};
    uint8_t _activeElementIndex = 0;
    uint8_t _activeElementCount = 0;
    uint8_t _profile {};
    bool _displayPendingForeground {};
    bool _displayPendingBackground {};
    //bool _blinkState {true};
    bool _backgroundRendered {false};
    bool _backgroundLayerSupported {false};

    // data 'local' to specific draw functions
    bool _HORIZON_SIDEBARS_RenderLevel {false};
    enum { AH_SIDEBAR_WIDTH_POS = 7, AH_SIDEBAR_HEIGHT_POS = 3 };
    int8_t _HORIZON_SIDEBARS_PosY {AH_SIDEBAR_HEIGHT_POS};
    uint8_t _RC_CHANNELS_channel {};
    enum {VERTICAL, HORIZONTAL} _STICK_OVERLAY_RenderPhase {VERTICAL};
    uint8_t _STICK_OVERLAY_Y {0};

    config_t _config {};
    std::array<uint8_t, OSD_ITEM_COUNT> _activeElements;
    std::bitset<OSD_ITEM_COUNT> _blinkBits {};

    // drawing functions
    static std::array<OSD_Elements::elementDrawFnPtr, OSD_ITEM_COUNT> DrawFunctions;
    static std::array<OSD_Elements::elementDrawFnPtr, OSD_ITEM_COUNT> DrawBackgroundFunctions;
};
