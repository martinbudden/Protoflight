#pragma once

#include "Targets.h"

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

struct osd_parameter_group_t {
    DisplayPortBase& displayPort;
    const AhrsMessageQueue& ahrs_message_queue;
    const FlightController& flightController;
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
2. Create a drawing function `void OSD::Elements::draw_MY_ELEMENT(DisplayPortBase& displayPort)`
   and optionally a background drawing function `void OSD::Elements::drawBackground_MY_ELEMENT(DisplayPortBase& displayPort)`.
3. Add the drawing function to the `DrawFunctions` array in `OSD_Elements::initDrawFunctions()`.
4. If you created a background drawing function then add it to the `DrawBackgroundFunctions` array.
5. Add `OSD_MY_ELEMENT` to the active elements in `OSD_Elements::addActiveElements()`.
*/
class OSD_Elements {
public:
    OSD_Elements(const OSD& osd);
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
    typedef void (OSD_Elements::*elementDrawFnPtr)(const osd_parameter_group_t& pg);

public:
#if defined(USE_OSD_HD)
    bool isSysOSD_Element(uint8_t index) { return (index >= OSD_SYS_GOGGLE_VOLTAGE) && (index <= OSD_SYS_FAN_SPEED); }
#else
    bool isSysOSD_Element(uint8_t index) { (void)index; return false; }
#endif

    const osd_elements_config_t& getConfig() const { return _config; }
    osd_elements_config_t& getConfig() { return _config; }
    void setConfig(const osd_elements_config_t& config);
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

    void setProfile(uint8_t profile);
    uint8_t getProfile() const { return _profile; }

    void addActiveElement(osd_elements_e element);
    void addActiveElements(const osd_parameter_group_t& pg);
    bool isRenderPending() const;
    uint8_t getActiveElementIndex() const { return _activeElementIndex; }
    uint8_t getActiveElementCount() const { return _activeElementCount; }

    void updateAttitude(float rollAngleDegrees, float pitchAngleDegrees, float yawAngleDegrees);
    bool drawNextActiveElement(const osd_parameter_group_t& pg);
    bool displayActiveElement(DisplayPortBase& displayPort);
    void drawActiveElementsBackground(const osd_parameter_group_t& pg);

    bool drawSpec(const osd_parameter_group_t& pg);

    bool drawSingleElement(const osd_parameter_group_t& pg, uint8_t elementIndex);
    bool drawSingleElementBackground(const osd_parameter_group_t& pg, uint8_t elementIndex);

    uint32_t displayWrite(DisplayPortBase& displayPort, const element_t& element, uint8_t x, uint8_t y, const char* s, uint8_t attr);
    uint32_t displayWrite(DisplayPortBase& displayPort, const element_t& element, uint8_t x, uint8_t y, uint8_t c, uint8_t attr);

// element drawing functions
    void formatDistanceString(char* buf, float distance, char leadingSymbol);
    static int printFloat(char* buffer, char leadingSymbol, float value, unsigned decimalPlaces, bool round, char trailingSymbol);
    void formatPID(const osd_parameter_group_t& pg, char* buf, const char* label, uint8_t axis);
    // no need to qualify functions with #if defined(USE_...) since linker will remove unused functions
    void draw_RSSI_VALUE(const osd_parameter_group_t& pg);
    void draw_MAIN_BATTERY_VOLTAGE(const osd_parameter_group_t& pg);
    void draw_CROSSHAIRS(const osd_parameter_group_t& pg); // only has background, but needs to be over other elements (like artificial horizon)
    void draw_ARTIFICIAL_HORIZON(const osd_parameter_group_t& pg);
    void draw_ITEM_TIMER(const osd_parameter_group_t& pg);
    void draw_FLYMODE(const osd_parameter_group_t& pg);
    void draw_THROTTLE_POS(const osd_parameter_group_t& pg);
    void draw_VTX_CHANNEL(const osd_parameter_group_t& pg);
    void draw_CURRENT_DRAW(const osd_parameter_group_t& pg);
    void draw_MAH_DRAWN(const osd_parameter_group_t& pg);
    void draw_GPS_SPEED(const osd_parameter_group_t& pg);
    void draw_GPS_SATS(const osd_parameter_group_t& pg);
    void draw_ALTITUDE(const osd_parameter_group_t& pg);
    void draw_ROLL_PIDS(const osd_parameter_group_t& pg);
    void draw_PITCH_PIDS(const osd_parameter_group_t& pg);
    void draw_YAW_PIDS(const osd_parameter_group_t& pg);
    void draw_POWER(const osd_parameter_group_t& pg);
    void draw_PID_RATE_PROFILE(const osd_parameter_group_t& pg);
    void draw_WARNINGS(const osd_parameter_group_t& pg);
    void draw_AVG_CELL_VOLTAGE(const osd_parameter_group_t& pg);
    void draw_GPS_LAT_LONG(const osd_parameter_group_t& pg);
    void draw_DEBUG(const osd_parameter_group_t& pg);
    void draw_PITCH_ANGLE(const osd_parameter_group_t& pg);
    void draw_ROLL_ANGLE(const osd_parameter_group_t& pg);
    void draw_MAIN_BATTERY_USAGE(const osd_parameter_group_t& pg);
    void draw_DISARMED(const osd_parameter_group_t& pg);
    void draw_HOME_DIRECTION(const osd_parameter_group_t& pg);
    void draw_HOME_DISTANCE(const osd_parameter_group_t& pg);
    void draw_NUMERICAL_HEADING(const osd_parameter_group_t& pg);
    void draw_NUMERICAL_VARIO(const osd_parameter_group_t& pg);
    void draw_COMPASS_BAR(const osd_parameter_group_t& pg);
    void draw_ESC_TEMPERATURE(const osd_parameter_group_t& pg);
    void draw_ESC_RPM(const osd_parameter_group_t& pg);
    void draw_REMAINING_TIME_ESTIMATE(const osd_parameter_group_t& pg);
    void draw_RTC_DATETIME(const osd_parameter_group_t& pg);
    void draw_ADJUSTMENT_RANGE(const osd_parameter_group_t& pg);
    void draw_CORE_TEMPERATURE(const osd_parameter_group_t& pg);
    void draw_ANTI_GRAVITY(const osd_parameter_group_t& pg);
    void draw_G_FORCE(const osd_parameter_group_t& pg);
    void draw_MOTOR_DIAGNOSTICS(const osd_parameter_group_t& pg);
    void draw_LOG_STATUS(const osd_parameter_group_t& pg);
    void draw_FLIP_ARROW(const osd_parameter_group_t& pg);
    void draw_LINK_QUALITY(const osd_parameter_group_t& pg);
    void draw_FLIGHT_DISTANCE(const osd_parameter_group_t& pg);
    void draw_STICK_OVERLAY(const osd_parameter_group_t& pg);
    void draw_PILOT_NAME(const osd_parameter_group_t& pg);
    void draw_ESC_RPM_FREQUENCY(const osd_parameter_group_t& pg);
    void draw_RATE_PROFILE_NAME(const osd_parameter_group_t& pg);
    void draw_PID_PROFILE_NAME(const osd_parameter_group_t& pg);
    void draw_PROFILE_NAME(const osd_parameter_group_t& pg);
    void draw_RSSI_DBM_VALUE(const osd_parameter_group_t& pg);
    void draw_RC_CHANNELS(const osd_parameter_group_t& pg);
    void draw_EFFICIENCY(const osd_parameter_group_t& pg);
    void draw_TOTAL_FLIGHTS(const osd_parameter_group_t& pg);
    void draw_UP_DOWN_REFERENCE(const osd_parameter_group_t& pg);
    void draw_TX_UPLINK_POWER(const osd_parameter_group_t& pg);
    void draw_WATT_HOURS_DRAWN(const osd_parameter_group_t& pg);
    void draw_AUX_VALUE(const osd_parameter_group_t& pg);
    void draw_READY_MODE(const osd_parameter_group_t& pg);
    void draw_RSNR_VALUE(const osd_parameter_group_t& pg);
    void draw_SYS(const osd_parameter_group_t& pg);
    void draw_GPS_LAP_TIME_CURRENT(const osd_parameter_group_t& pg);
    void draw_GPS_LAP_TIME_PREVIOUS(const osd_parameter_group_t& pg);
    void draw_GPS_LAP_TIME_BEST3(const osd_parameter_group_t& pg);
    void draw_DEBUG2(const osd_parameter_group_t& pg);
    void draw_CUSTOM_MSG(const osd_parameter_group_t& pg);
    void draw_LIDAR_DISTANCE(const osd_parameter_group_t& pg);
// element background drawing functions
    void drawBackground_HORIZON_SIDEBARS(const osd_parameter_group_t& pg);
    void drawBackground_CRAFT_NAME(const osd_parameter_group_t& pg);
    void drawBackground_STICK_OVERLAY(const osd_parameter_group_t& pg);
    void drawBackground_PILOT_NAME(const osd_parameter_group_t& pg);
    void drawBackground_CAMERA_FRAME(const osd_parameter_group_t& pg);
private:
    const OSD& _osd;
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

    osd_elements_config_t _config {};
    std::array<uint8_t, OSD_ELEMENT_COUNT> _activeElements;
    std::bitset<OSD_ELEMENT_COUNT> _blinkBits {};
private:
    // drawing functions
    static std::array<OSD_Elements::elementDrawFnPtr, OSD_ELEMENT_COUNT> DrawFunctions;
    static std::array<OSD_Elements::elementDrawFnPtr, OSD_ELEMENT_COUNT> DrawBackgroundFunctions;
};
