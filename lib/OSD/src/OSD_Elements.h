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

enum  osd_items_e {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATT_VOLTAGE,
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
    OSD_GPS_SPEED,
    OSD_GPS_SATS,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_PID_RATE_PROFILE,
    OSD_WARNINGS,
    OSD_AVG_CELL_VOLTAGE,
    OSD_GPS_LON,
    OSD_GPS_LAT,
    OSD_DEBUG,
    OSD_PITCH_ANGLE,
    OSD_ROLL_ANGLE,
    OSD_MAIN_BATT_USAGE,
    OSD_DISARMED,
    OSD_HOME_DIR,
    OSD_HOME_DIST,
    OSD_NUMERICAL_HEADING,
    OSD_NUMERICAL_VARIO,
    OSD_COMPASS_BAR,
    OSD_ESC_TMP,
    OSD_ESC_RPM,
    OSD_REMAINING_TIME_ESTIMATE,
    OSD_RTC_DATETIME,
    OSD_ADJUSTMENT_RANGE,
    OSD_CORE_TEMPERATURE,
    OSD_ANTI_GRAVITY,
    OSD_G_FORCE,
    OSD_MOTOR_DIAG,
    OSD_LOG_STATUS,
    OSD_FLIP_ARROW,
    OSD_LINK_QUALITY,
    OSD_FLIGHT_DIST,
    OSD_STICK_OVERLAY_LEFT,
    OSD_STICK_OVERLAY_RIGHT,
    OSD_PILOT_NAME,
    OSD_ESC_RPM_FREQ,
    OSD_RATE_PROFILE_NAME,
    OSD_PID_PROFILE_NAME,
    OSD_PROFILE_NAME,
    OSD_RSSI_DBM_VALUE,
    OSD_RC_CHANNELS,
    OSD_CAMERA_FRAME,
    OSD_EFFICIENCY,
    OSD_TOTAL_FLIGHTS,
    OSD_UP_DOWN_REFERENCE,
    OSD_TX_UPLINK_POWER,
    OSD_WATT_HOURS_DRAWN,
    OSD_AUX_VALUE,
    OSD_READY_MODE,
    OSD_RSNR_VALUE,
    OSD_SYS_GOGGLE_VOLTAGE,
    OSD_SYS_VTX_VOLTAGE,
    OSD_SYS_BITRATE,
    OSD_SYS_DELAY,
    OSD_SYS_DISTANCE,
    OSD_SYS_LQ,
    OSD_SYS_GOGGLE_DVR,
    OSD_SYS_VTX_DVR,
    OSD_SYS_WARNINGS,
    OSD_SYS_VTX_TEMP,
    OSD_SYS_FAN_SPEED,
    OSD_GPS_LAP_TIME_CURRENT,
    OSD_GPS_LAP_TIME_PREVIOUS,
    OSD_GPS_LAP_TIME_BEST3,
    OSD_DEBUG2,
    OSD_CUSTOM_MSG0,
    OSD_CUSTOM_MSG1,
    OSD_CUSTOM_MSG2,
    OSD_CUSTOM_MSG3,
    OSD_LIDAR_DIST,
    OSD_ITEM_COUNT // MUST BE LAST
};

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
    bool isSysOSD_Element(uint8_t index) { return (index >= OSD_SYS_GOGGLE_VOLTAGE) && (index <= OSD_SYS_FAN_SPEED); }

    const config_t& getConfig() const { return _config; }
    config_t& getConfig() { return _config; }
    void setConfig(const config_t& config);
    void setDefaultConfig(uint8_t rowCount, uint8_t columnCount);

    static constexpr uint16_t ELEMENT_BITS_POS = 14;
    static constexpr uint32_t ELEMENT_TYPE_MASK = 0b1100'0000'0000'0000U;  // bits 14-15
    static element_type_e ELEMENT_TYPE(uint32_t x) { return static_cast<element_type_e>((x & ELEMENT_TYPE_MASK) >> ELEMENT_BITS_POS); }

    enum { PROFILE_COUNT = 2 };
    static constexpr uint16_t PROFILE_BITS_POS = 12;
    static constexpr uint16_t PROFILE_MASK = 0b0011'0000'0000'0000U;
    static uint16_t profileFlag(uint16_t x) { return 1U << (x - 1 + PROFILE_BITS_POS); }
    static bool elementVisible(uint16_t value, uint16_t profile) { return ((value & PROFILE_MASK) >> PROFILE_BITS_POS) & (1 << profile); }

    static constexpr uint16_t XY_POSITION_BITS = 6;       // 6 bits gives a range 0-63
    static constexpr uint16_t XY_POSITION_MASK = 0b11'1111U;

    static uint8_t OSD_X(uint16_t x) { return x & XY_POSITION_MASK; }
    static uint8_t OSD_Y(uint16_t x) { return (x >> XY_POSITION_BITS) & XY_POSITION_MASK; }
    static uint16_t OSD_POS(uint8_t x, uint8_t y) { return (x & XY_POSITION_MASK) | ((y & XY_POSITION_MASK) << XY_POSITION_BITS); }

    void addActiveElement(osd_items_e element);
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
    void drawPIDsRoll(DisplayPortBase& displayPort);
    void drawPIDsPitch(DisplayPortBase& displayPort);
    void drawPIDsYaw(DisplayPortBase& displayPort);
    void drawRSSI(DisplayPortBase& displayPort);
    void drawMainBatteryVoltage(DisplayPortBase& displayPort);
    void drawCrosshairs(DisplayPortBase& displayPort);  // only has background, but needs to be over other elements (like artificial horizon)
    void drawArtificialHorizon(DisplayPortBase& displayPort);
    void drawDebug(DisplayPortBase& displayPort);
    void drawAngleRoll(DisplayPortBase& displayPort);
    void drawAnglePitch(DisplayPortBase& displayPort);
    void drawDisarmed(DisplayPortBase& displayPort);
    void drawRC_Channels(DisplayPortBase& displayPort);
    void drawDebug2(DisplayPortBase& displayPort);
    void drawNumericalHeading(DisplayPortBase& displayPort);
    void drawWarnings(DisplayPortBase& displayPort);
// element background drawing functions
    void drawBackgroundHorizonSidebars(DisplayPortBase& displayPort);
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
    bool _displayPendingForeground {};
    bool _displayPendingBackground {};
    //bool _blinkState {true};
    bool _backgroundRendered {false};
    bool _backgroundLayerSupported {false};
    bool _sideBarRenderLevel {false};
    enum { AH_SIDEBAR_WIDTH_POS = 7, AH_SIDEBAR_HEIGHT_POS = 3 };
    int8_t _sidbarPosY {AH_SIDEBAR_HEIGHT_POS};
    uint8_t _rcChannel {};
    uint8_t _profile {};

    config_t _config {};
    std::array<uint8_t, OSD_ITEM_COUNT> _activeElements;
    std::bitset<OSD_ITEM_COUNT> _blinkBits {};

    static std::array<OSD_Elements::elementDrawFnPtr, OSD_ITEM_COUNT> DrawFunctions;
    static std::array<OSD_Elements::elementDrawFnPtr, OSD_ITEM_COUNT> DrawBackgroundFunctions;
};
