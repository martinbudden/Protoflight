#pragma once

#include "CMSX.h"

#include <TimeMicroseconds.h>
#include <array>
#include <cstdint>

struct receiver_controls_pwm_t;
class Cockpit;
class DisplayPortBase;
class IMU_Base;
class IMU_Filters;
class NonVolatileStorage;
class OSD;
class RcModes;
class ReceiverBase;


/*!
Custom Menu System.
*/
class CMS {
public:
    CMS(DisplayPortBase* displayPort, Cockpit& cockpit, const ReceiverBase& receiver, RcModes& rc_modes, IMU_Filters& imuFilters, IMU_Base& imu, NonVolatileStorage& nvs, VTX* vtx);
    void init();
private:
    // CMS is not copyable or moveable
    CMS(const CMS&) = delete;
    CMS& operator=(const CMS&) = delete;
    CMS(CMS&&) = delete;
    CMS& operator=(CMS&&) = delete;
public:
    enum { MAX_DISPLAY_PORT_COUNT = 4 };
    enum { HEARTBEAT_INTERVAL_MS = 500 };
public:
    struct config_t {
        uint8_t filler;
    };
public:
    const config_t& getConfig() const { return _config; }
    void setConfig(const config_t& config);

    void updateCMS(uint32_t currentTimeUs, uint32_t timeMicrosecondsDelta); //!< CMS Task function, called by Task

    uint16_t handleKeyWithRepeat(CMSX::key_e key, size_t repeatCount);
    bool scanKeys(int32_t timeDelta, const receiver_controls_pwm_t& controls, bool isArmed);
    void setExternKey(CMSX::key_e externKey);

    DisplayPortBase* displayPortSelectNext();
    void setDisplayPort(DisplayPortBase* displayPort) { _displayPort = displayPort; }
    bool displayPortSelect(const DisplayPortBase* displayPort);

    const ReceiverBase& getReceiver() const { return _receiver; }
private:
    DisplayPortBase* _displayPort;
    CMSX _cmsx;
    const Cockpit& _cockpit;
    const ReceiverBase& _receiver;
    const RcModes& _rc_modes;
    config_t _config {};
    int32_t _keyDelayMs {CMSX::BUTTON_TIME_MS};
    uint32_t _lastCalledMs {};
    uint32_t _lastHeartbeatTimeMs {};
    uint32_t _deviceCount {0};
    int32_t _currentDeviceIndex {-1};
    uint32_t _holdCount {1};
    uint32_t _repeatCount {1};
    uint32_t _repeatBase {0};
    CMSX::key_e _externKey {};

    std::array<DisplayPortBase*, MAX_DISPLAY_PORT_COUNT> _displayPorts {};
};
