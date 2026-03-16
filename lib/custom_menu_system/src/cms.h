#pragma once

#include "cmsx.h"

#include <array>
#include <cstdint>
#include <time_microseconds.h>

struct receiver_controls_pwm_t;
class DisplayPortBase;


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
    const config_t& get_config() const { return _config; }
    void set_config(const config_t& config);

    void update_cms(cms_context_t& ctx, uint32_t current_time_us, uint32_t time_microseconds_delta); //!< CMS Task function, called by Task

    uint16_t handle_key_with_repeat(cms_context_t& ctx, CMSX::key_e key, size_t repeat_count);
    bool scan_keys(cms_context_t& ctx, int32_t time_delta, const receiver_controls_pwm_t& controls, bool is_armed);
    void set_extern_key(CMSX::key_e extern_key);
private:
    CMSX _cmsx;
    config_t _config {};
    int32_t _key_delay_ms {CMSX::BUTTON_TIME_MS};
    uint32_t _last_called_ms {};
    uint32_t _last_heartbeat_time_ms {};
    uint32_t _device_count {0};
    int32_t _current_device_index {-1};
    uint32_t _hold_count {1};
    uint32_t _repeat_count {1};
    uint32_t _repeat_base {0};
    CMSX::key_e _extern_key {};

    std::array<DisplayPortBase*, MAX_DISPLAY_PORT_COUNT> _display_ports {};
};
