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

#ifdef USE_OSD_PROFILES
    enum { OSD_PROFILE_COUNT = 3 };
#else
    enum { OSD_PROFILE_COUNT = 1 };
#endif


// Character coordinate
#define OSD_POSITION_BITS       5       // 5 bits gives a range 0-31
#define OSD_POSITION_BIT_XHD    10      // extra bit used to extend X range in a backward compatible manner for HD displays
#define OSD_POSITION_XHD_MASK   (1 << OSD_POSITION_BIT_XHD)
#define OSD_POSITION_XY_MASK    ((1 << OSD_POSITION_BITS) - 1)
#define OSD_POS(x, y)  (((x) & OSD_POSITION_XY_MASK)                    \
                        | (((x) << (OSD_POSITION_BIT_XHD - OSD_POSITION_BITS)) & OSD_POSITION_XHD_MASK) \
                        | (((y) & OSD_POSITION_XY_MASK) << OSD_POSITION_BITS)) \
    /**/
#define OSD_X(x)      (((x) & OSD_POSITION_XY_MASK) | (((x) & OSD_POSITION_XHD_MASK) >> (OSD_POSITION_BIT_XHD - OSD_POSITION_BITS)))
#define OSD_Y(x)      (((x) >> OSD_POSITION_BITS) & OSD_POSITION_XY_MASK)

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
    OSD_Elements(OSD& osd, const FlightController& flightController, const Debug& debug);
    void init(bool backgroundLayerFlag);
    void initDrawFunctions();
public:
    enum element_type_e{
        OSD_ELEMENT_TYPE_1 = 0,
        OSD_ELEMENT_TYPE_2,
        OSD_ELEMENT_TYPE_3,
        OSD_ELEMENT_TYPE_4
    };
    struct config_t {
        std::array<uint16_t, OSD_ITEM_COUNT> item_pos;
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
        bool drawElement;
        bool rendered;
        uint8_t attr;
    };
    typedef void (OSD_Elements::*elementDrawFn)(element_t& element);

    static constexpr uint32_t OSD_TYPE_MASK = 0xC000;  // bits 14-15
    static uint32_t OSD_TYPE(uint32_t x) { return (x & OSD_TYPE_MASK) >> 14; }
public:
    bool isSysOSD_Element(uint8_t index) { return (index >= OSD_SYS_GOGGLE_VOLTAGE) && (index <= OSD_SYS_FAN_SPEED); }

    const config_t& getConfig() const { return _config; }
    config_t& getConfig() { return _config; }
    void setConfig(const config_t& config);
    void setConfigDefaults();

    int convertTemperatureToSelectedUnit(int temperatureCelsius);
    void formatDistanceString(char *result, int distance, char leadingSymbol);
    bool formatRtcDateTime(char *buffer);
    //void formatTime(char * buff, _timer_precision_e precision, timeUs32_t time);
    void formatTimer(char *buff, bool showSymbol, bool usePrecision, int timerIndex);
    float getMetersToSelectedUnit(int32_t meters);
    char getMetersToSelectedUnitSymbol();
    int32_t getSpeedToSelectedUnit(int32_t value);
    char getSpeedToSelectedUnitSymbol();
    char getTemperatureSymbolForSelectedUnit();
    void addActiveElements();
    bool isRenderPending() const;
    uint8_t getActiveElement() const;
    uint8_t getActiveElementCount() const;
    bool drawNextActiveElement(DisplayPortBase& displayPort);
    bool displayActiveElement(DisplayPortBase& displayPort);
    void drawActiveElementsBackground(DisplayPortBase& displayPort);

    void syncBlink(timeUs32_t currentTimeUs);
    void resetAlarms();
    void updateAlarms();
    bool elementsNeedAccelerometer();
    bool drawSpec(DisplayPortBase *displayPort);

    bool drawSingleElement(DisplayPortBase& displayPort, uint8_t elementIndex);
    bool drawSingleElementBackground(DisplayPortBase& displayPort, uint8_t elementIndex);

    int displayWrite(DisplayPortBase& displayPort, const element_t& element, uint8_t x, uint8_t y, uint8_t attr, const char *s);
    int displayWrite(DisplayPortBase& displayPort, const element_t& element, uint8_t x, uint8_t y, uint8_t attr, char c);

// element drawing functions
    void formatPID(char * buf, const char * label, uint8_t axis);
    void drawPIDsRoll(element_t& element);
    void drawPIDsPitch(element_t& element);
    void drawPIDsYaw(element_t& element);
    void drawRSSI(element_t& element);
    void drawMainBatteryVoltage(element_t& element);
    void drawCrosshairs(element_t& element);  // only has background, but needs to be over other elements (like artificial horizon)
    void drawArtificialHorizon(element_t& element);
    void drawDebug(element_t& element);
    void drawDebug2(element_t& element);
// element background drawing functions
    void drawBackgroundHorizonSidebars(element_t& element);
private:
    OSD& _osd;
    const FlightController& _flightController;
    const Debug& _debug;
    element_t _activeElement {};
    config_t _config {};
    bool _displayPendingForeground {};
    bool _displayPendingBackground {};
    bool blinkState {true};
    uint8_t _activeElementIndex = 0;
    uint8_t _activeElementCount = 0;
    bool _backgroundRendered {false};
    bool _backgroundLayerSupported {false};
    bool _sideBarRenderLevel {false};
    enum { AH_SIDEBAR_WIDTH_POS = 7, AH_SIDEBAR_HEIGHT_POS = 3 };
    int8_t _sidbarPosY {AH_SIDEBAR_HEIGHT_POS};

    std::array<uint8_t, OSD_ITEM_COUNT> _activeOsdElementArray;
    std::bitset<OSD_ITEM_COUNT> _blinkBits {};

    static std::array<OSD_Elements::elementDrawFn, OSD_ITEM_COUNT> elementDrawFunctions;
    static std::array<OSD_Elements::elementDrawFn, OSD_ITEM_COUNT> elementDrawBackgroundFunctions;
};
