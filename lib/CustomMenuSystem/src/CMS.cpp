#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "DisplayPortBase.h"
#include "MspBox.h"
#include "RC_Modes.h"
#include <receiver_base.h>

#if (__cplusplus >= 202002L)
#include <ranges>
#endif


CMS::CMS() : // NOLINT(hicpp-use-equals-default,modernize-use-equals-default)
    _cmsx(*this)
{
}

void CMS::init()
{
    _deviceCount = 0;
    _currentDeviceIndex = -1;

}

void CMS::setConfig(const config_t& config)
{
    _config = config;
}

void CMS::updateCMS(cms_parameter_group_t& pg, uint32_t currentTimeUs, uint32_t time_microseconds_delta) // NOLINT(readability-function-cognitive-complexity)
{
    (void)pg;
    (void)time_microseconds_delta;

    if (pg.rc_modes.is_mode_active(MspBox::BOX_PARALYZE)) {
        return;
    }

    const uint32_t currentTimeMs = currentTimeUs / 1000;

    const receiver_controls_pwm_t controls = pg.receiver.get_controls_pwm();
    const bool isArmed = pg.cockpit.isArmed();
    if (_cmsx.isInMenu()) {
        if (scanKeys(pg, static_cast<int32_t>(currentTimeMs - _lastCalledMs), controls, isArmed)) {
            _cmsx.drawMenu(pg, currentTimeUs);
            pg.displayPort.commitTransaction();
        }
        if (currentTimeMs > _lastHeartbeatTimeMs + HEARTBEAT_INTERVAL_MS) {
            // Heart beat for external CMS display device @500ms, timeout @1000ms
            pg.displayPort.heartbeat();
            _lastHeartbeatTimeMs = currentTimeMs;
        }
    } else {
        // Detect menu invocation
        if (!isArmed && !pg.rc_modes.is_mode_active(MspBox::BOX_STICK_COMMAND_DISABLE)) {
#if defined(LIBRARY_RECEIVER_USE_ESPNOW )
            if (RcModes::pwm_is_low(controls.yaw) && RcModes::pwm_is_high(controls.pitch)) {
#else
            if (RcModes::pwm_is_low(controls.yaw) && RcModes::pwm_is_high(controls.pitch) && RcModes::pwm_is_mid(controls.throttle)) {
#endif
                pg.displayPort.beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
                _cmsx.menuOpen(pg);
                _cmsx.drawMenu(pg, currentTimeUs);
                pg.displayPort.commitTransaction();
                _keyDelayMs = CMSX::BUTTON_PAUSE_MS; // Tends to overshoot if BUTTON_TIME_MS used
            }
        }
    }
    // Some commands, notably flash erase, take a long time, so currentTimeMs is not an accurate value for _lastCalledMs
    _lastCalledMs = time_ms();
}

void CMS::setExternKey(CMSX::key_e externKey)
{
    if (_externKey == CMSX::KEY_NONE) {
        _externKey = externKey;
    }
}

uint16_t CMS::handleKeyWithRepeat(cms_parameter_group_t& pg, CMSX::key_e key, size_t repeatCount)
{
    uint16_t ret = 0;
#if (__cplusplus >= 202002L)
    for ([[maybe_unused]] auto _ : std::views::iota(size_t{0}, repeatCount)) {
#else
    for (size_t ii = 0; ii < repeatCount ; ++ii) {
#endif
        ret = _cmsx.handleKey(pg, key);
    }
    return ret;
}

bool CMS::scanKeys(cms_parameter_group_t& pg, int32_t timeDelta, const receiver_controls_pwm_t& controls, bool isArmed) // NOLINT(readability-function-cognitive-complexity)
{
    if (_externKey != CMSX::KEY_NONE) {
        _keyDelayMs = _cmsx.handleKey(pg, _externKey);
        _externKey = CMSX::KEY_NONE;
        pg.displayPort.beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        return true;
    }

    CMSX::key_e key = CMSX::KEY_NONE;
    if (isArmed == false && RcModes::pwm_is_mid(controls.throttle) && RcModes::pwm_is_low(controls.yaw) && RcModes::pwm_is_high(controls.pitch)) {
        key = CMSX::KEY_MENU;
    } else if (RcModes::pwm_is_high(controls.pitch)) {
        key = CMSX::KEY_UP;
    } else if (RcModes::pwm_is_low(controls.pitch)) {
        key = CMSX::KEY_DOWN;
    } else if (RcModes::pwm_is_high(controls.roll)) {
        key = CMSX::KEY_RIGHT;
    } else if (RcModes::pwm_is_low(controls.roll)) {
        key = CMSX::KEY_LEFT;
    } else if (RcModes::pwm_is_high(controls.yaw)) {
        key = CMSX::KEY_ESC;
    } else if (RcModes::pwm_is_low(controls.yaw)) {
        key = CMSX::KEY_SAVE_MENU;
    }
    if (key == CMSX::KEY_NONE) {
        // No 'key' pressed, reset repeat control
        _holdCount = 1;
        _repeatCount = 1;
        _repeatBase = 0;
    } else {
        // The 'key' is being pressed; keep counting
        ++_holdCount;
    }
    if (_keyDelayMs > 0) {
        _keyDelayMs -= timeDelta;
        return false;
    }
    if (key) {
        pg.displayPort.beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        _keyDelayMs = handleKeyWithRepeat(pg, key, _repeatCount);
        // Key repeat effect is implemented in two phases.
        // First phase is to decrease keyDelayMs reciprocal to hold time.
        // When keyDelayMs reached a certain limit (scheduling interval),
        // repeat rate will not raise anymore, so we call key handler
        // multiple times (repeatCount).
        //
        // XXX Caveat: Most constants are adjusted pragmatically.
        // XXX Rewrite this someday, so it uses actual hold time instead
        // of holdCount, which depends on the scheduling interval.
        if (((key == CMSX::KEY_LEFT) || (key == CMSX::KEY_RIGHT)) && (_holdCount > 20)) {
            // Decrease keyDelayMs reciprocally
            _keyDelayMs /= static_cast<int32_t>(_holdCount - 20);
            // When we reach the scheduling limit,
            if (_keyDelayMs <= 50) {
                // start calling handler multiple times.
                if (_repeatBase == 0) {
                    _repeatBase = _holdCount;
                }
                _repeatCount += (_holdCount - _repeatBase) / 5;
                if (_repeatCount > 5) {
                    _repeatCount = 5;
                }
            }
        }
        return true;
    }
    return false;
}

#if false
DisplayPortBase* CMS::displayPortSelectNext()
{
    if (_deviceCount == 0) {
        return nullptr;
    }
    ++_currentDeviceIndex;
    if (_currentDeviceIndex == static_cast<int32_t>(_deviceCount)) {
        _currentDeviceIndex = 0;
    }
    return _displayPorts[static_cast<size_t>(_currentDeviceIndex)];
}

bool CMS::displayPortSelect(const DisplayPortBase* displayPort)
{
#if (__cplusplus >= 202002L)
    for ([[maybe_unused]] auto _ : std::views::iota(size_t{0}, size_t{_deviceCount})) { // cppcheck-suppress useStlAlgorithm
#else
    for (size_t ii = 0; ii < _deviceCount; ++ii) {
#endif
        if (displayPortSelectNext() == displayPort) {
            return true;
        }
    }
    return false;
}
#endif
