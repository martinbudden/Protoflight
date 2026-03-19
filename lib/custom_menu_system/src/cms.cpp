#include "cms.h"
#include "cms_types.h"
#include "cmsx.h"
#include "cockpit.h"
#include "display_port_base.h"
#include "msp_box.h"
#include "rc_modes.h"

#if (__cplusplus >= 202002L)
#include <ranges>
#endif

#include <receiver_base.h>


CMS::CMS() : // NOLINT(hicpp-use-equals-default,modernize-use-equals-default)
    _cmsx(*this)
{
}

void CMS::init()
{
    _device_count = 0;
    _current_device_index = -1;

}

void CMS::set_config(const cms_config_t& config)
{
    _config = config;
}

void CMS::update_cms(cms_context_t& ctx, uint32_t current_time_us, uint32_t time_microseconds_delta) // NOLINT(readability-function-cognitive-complexity)
{
    (void)ctx;
    (void)time_microseconds_delta;

    if (ctx.rc_modes.is_mode_active(MspBox::BOX_PARALYZE)) {
        return;
    }

    const uint32_t currentTimeMs = current_time_us / 1000;

    const receiver_controls_pwm_t controls = ctx.receiver.get_controls_pwm();
    const bool is_armed = ctx.cockpit.is_armed();
    if (_cmsx.is_in_menu()) {
        if (scan_keys(ctx, static_cast<int32_t>(currentTimeMs - _last_called_ms), controls, is_armed)) {
            _cmsx.draw_menu(ctx, current_time_us);
            ctx.display_port.commit_transaction();
        }
        if (currentTimeMs > _last_heartbeat_time_ms + HEARTBEAT_INTERVAL_MS) {
            // Heart beat for external CMS display device @500ms, timeout @1000ms
            ctx.display_port.heartbeat();
            _last_heartbeat_time_ms = currentTimeMs;
        }
    } else {
        // Detect menu invocation
        if (!is_armed && !ctx.rc_modes.is_mode_active(MspBox::BOX_STICK_COMMAND_DISABLE)) {
#if defined(LIBRARY_RECEIVER_USE_ESPNOW )
            if (RcModes::pwm_is_low(controls.yaw) && RcModes::pwm_is_high(controls.pitch)) {
#else
            if (RcModes::pwm_is_low(controls.yaw) && RcModes::pwm_is_high(controls.pitch) && RcModes::pwm_is_mid(controls.throttle)) {
#endif
                ctx.display_port.begin_transaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
                _cmsx.menu_open(ctx);
                _cmsx.draw_menu(ctx, current_time_us);
                ctx.display_port.commit_transaction();
                _key_delay_ms = CMSX::BUTTON_PAUSE_MS; // Tends to overshoot if BUTTON_TIME_MS used
            }
        }
    }
    // Some commands, notably flash erase, take a long time, so currentTimeMs is not an accurate value for _last_called_ms
    _last_called_ms = time_ms();
}

void CMS::set_extern_key(cmsx_key_e extern_key)
{
    if (_extern_key == CMSX_KEY_NONE) {
        _extern_key = extern_key;
    }
}

uint16_t CMS::handle_key_with_repeat(cms_context_t& ctx, cmsx_key_e key, size_t repeat_count)
{
    uint16_t ret = 0;
#if (__cplusplus >= 202002L)
    for ([[maybe_unused]] auto _ : std::views::iota(size_t{0}, repeat_count)) {
#else
    for (size_t ii = 0; ii < repeat_count; ++ii) {
#endif
        ret = _cmsx.handle_key(ctx, key);
    }
    return ret;
}

bool CMS::scan_keys(cms_context_t& ctx, int32_t time_delta, const receiver_controls_pwm_t& controls, bool is_armed) // NOLINT(readability-function-cognitive-complexity)
{
    if (_extern_key != CMSX_KEY_NONE) {
        _key_delay_ms = _cmsx.handle_key(ctx, _extern_key);
        _extern_key = CMSX_KEY_NONE;
        ctx.display_port.begin_transaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        return true;
    }

    cmsx_key_e key = CMSX_KEY_NONE;
    if (is_armed == false && RcModes::pwm_is_mid(controls.throttle) && RcModes::pwm_is_low(controls.yaw) && RcModes::pwm_is_high(controls.pitch)) {
        key = CMSX_KEY_MENU;
    } else if (RcModes::pwm_is_high(controls.pitch)) {
        key = CMSX_KEY_UP;
    } else if (RcModes::pwm_is_low(controls.pitch)) {
        key = CMSX_KEY_DOWN;
    } else if (RcModes::pwm_is_high(controls.roll)) {
        key = CMSX_KEY_RIGHT;
    } else if (RcModes::pwm_is_low(controls.roll)) {
        key = CMSX_KEY_LEFT;
    } else if (RcModes::pwm_is_high(controls.yaw)) {
        key = CMSX_KEY_ESC;
    } else if (RcModes::pwm_is_low(controls.yaw)) {
        key = CMSX_KEY_SAVE_MENU;
    }
    if (key == CMSX_KEY_NONE) {
        // No 'key' pressed, reset repeat control
        _hold_count = 1;
        _repeat_count = 1;
        _repeat_base = 0;
    } else {
        // The 'key' is being pressed; keep counting
        ++_hold_count;
    }
    if (_key_delay_ms > 0) {
        _key_delay_ms -= time_delta;
        return false;
    }
    if (key) {
        ctx.display_port.begin_transaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        _key_delay_ms = handle_key_with_repeat(ctx, key, _repeat_count);
        // Key repeat effect is implemented in two phases.
        // First phase is to decrease key_delay_ms reciprocal to hold time.
        // When key_delay_ms reached a certain limit (scheduling interval),
        // repeat rate will not raise anymore, so we call key handler
        // multiple times (repeat_count).
        //
        // XXX Caveat: Most constants are adjusted pragmatically.
        // XXX Rewrite this someday, so it uses actual hold time instead
        // of hold_count, which depends on the scheduling interval.
        if (((key == CMSX_KEY_LEFT) || (key == CMSX_KEY_RIGHT)) && (_hold_count > 20)) {
            // Decrease key_delay_ms reciprocally
            _key_delay_ms /= static_cast<int32_t>(_hold_count - 20);
            // When we reach the scheduling limit,
            if (_key_delay_ms <= 50) {
                // start calling handler multiple times.
                if (_repeat_base == 0) {
                    _repeat_base = _hold_count;
                }
                _repeat_count += (_hold_count - _repeat_base) / 5;
                if (_repeat_count > 5) {
                    _repeat_count = 5;
                }
            }
        }
        return true;
    }
    return false;
}

#if false
DisplayPortBase* CMS::display_portSelectNext()
{
    if (_device_count == 0) {
        return nullptr;
    }
    ++_current_device_index;
    if (_current_device_index == static_cast<int32_t>(_device_count)) {
        _current_device_index = 0;
    }
    return _display_ports[static_cast<size_t>(_current_device_index)];
}

bool CMS::display_portSelect(const DisplayPortBase* display_port)
{
#if (__cplusplus >= 202002L)
    for ([[maybe_unused]] auto _ : std::views::iota(size_t{0}, size_t{_device_count})) { // cppcheck-suppress useStlAlgorithm
#else
    for (size_t ii = 0; ii < _device_count; ++ii) {
#endif
        if (display_portSelectNext() == display_port) {
            return true;
        }
    }
    return false;
}
#endif
