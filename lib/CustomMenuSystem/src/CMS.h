#pragma once

#include "CMSX.h"

#include <array>
#include <cstdint>
#include <time_microseconds.h>

struct receiver_controls_pwm_t;
class Cockpit;
class DisplayPortBase;
class ImuBase;
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
    CMS();
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

    void updateCMS(cms_parameter_group_t& pg, uint32_t currentTimeUs, uint32_t time_microseconds_delta); //!< CMS Task function, called by Task

    uint16_t handleKeyWithRepeat(cms_parameter_group_t& pg, CMSX::key_e key, size_t repeatCount);
    bool scanKeys(cms_parameter_group_t& pg, int32_t timeDelta, const receiver_controls_pwm_t& controls, bool isArmed);
    void setExternKey(CMSX::key_e externKey);
private:
    CMSX _cmsx;
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
