#pragma once

#include "CMSX.h"

#include <TimeMicroseconds.h>
#include <array>
#include <cstdint>

class Cockpit;
class DisplayPortBase;
class IMU_Filters;
class OSD;
class ReceiverBase;

class CMS {
public:
    CMS(DisplayPortBase* displayPort, const ReceiverBase& receiver, Cockpit& cockpit, IMU_Filters& imuFilters, OSD* osd);
    void init();
private:
    // CMS is not copyable or moveable
    CMS(const CMS&) = delete;
    CMS& operator=(const CMS&) = delete;
    CMS(CMS&&) = delete;
    CMS& operator=(CMS&&) = delete;
public:
    enum { BUTTON_TIME_MS = 250 };
    enum { BUTTON_PAUSE_MS = 500 };
    enum key_e {
        KEY_NONE,
        KEY_UP,
        KEY_DOWN,
        KEY_LEFT,
        KEY_RIGHT,
        KEY_ESC,
        KEY_MENU,
        KEY_SAVEMENU,
    };
    enum { MAX_DISPLAY_PORT_COUNT = 4 };
    enum { HEARTBEAT_INTERVAL_MS = 500 };
public:
    struct config_t {
        uint8_t filler;
    };
public:
    const config_t& getConfig() const { return _config; }
    void setConfig(const config_t& config);

    IMU_Filters& getIMU_Filters() { return _imuFilters; };

    void updateCMS(uint32_t currentTimeUs, uint32_t timeMicrosecondsDelta); //!< CMS Task function, called by Task

    uint32_t handleKey(key_e key);
    uint32_t scanKeys(uint32_t currentTimeMs, uint32_t lastCalledMs, uint32_t rcDelayMs);
    void setExternKey(key_e externKey);


    void drawMenu(uint32_t currentTimeUs);
    void menuOpen();

    DisplayPortBase* displayPortSelectNext();
    bool displayPortSelect(const DisplayPortBase* displayPort);
    static bool pwmIsHigh(uint16_t x) { return x > 1750; }
    static bool pwmIsLow(uint16_t x) { return x < 1250; }
    static bool pwmIsMid(uint16_t x) { return (x > 1250) && (x <1750); }

    Cockpit& getCockpit() { return _cockpit; }
private:
    DisplayPortBase* _displayPort;
    CMSX _cmsx;
    const ReceiverBase& _receiver;
    Cockpit& _cockpit;
    IMU_Filters& _imuFilters;
    OSD* _osd;
    config_t _config {};
    int32_t _rcDelayMs {BUTTON_TIME_MS};
    uint32_t _lastCalledMs {};
    uint32_t _lastHeartbeatTimeMs {};
    uint32_t _deviceCount {0};
    int32_t _currentDeviceIndex {-1};

    key_e _externKey {};
    std::array<DisplayPortBase*, MAX_DISPLAY_PORT_COUNT> _displayPorts {};
};
