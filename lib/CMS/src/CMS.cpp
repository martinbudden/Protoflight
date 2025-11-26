#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "DisplayPortBase.h"

#include <MSP_Box.h>
#include <ReceiverBase.h>


CMS::CMS(DisplayPortBase* displayPort, const ReceiverBase& receiver, Cockpit& cockpit, IMU_Filters& imuFilters, IMU_Base& imu, NonVolatileStorage& nvs, OSD* osd, VTX_Base* vtx) :
    _displayPort(displayPort),
    _cmsx(*this, imuFilters, imu, nvs, vtx),
    _receiver(receiver),
    _cockpit(cockpit),
    _osd(osd)
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

void CMS::setArmingDisabled()
{
    _cockpit.setArmingDisabledFlag(ARMING_DISABLED_CMS_MENU);
}

void CMS::clearArmingDisabled()
{
    _cockpit.clearArmingDisabledFlag(ARMING_DISABLED_CMS_MENU);
}

void CMS::updateCMS(uint32_t currentTimeUs, uint32_t timeMicrosecondsDelta) // NOLINT(readability-function-cognitive-complexity)
{
    (void)timeMicrosecondsDelta;

    if (_cockpit.isRcModeActive(MSP_Box::BOX_PARALYZE)) {
        return;
    }

    const uint32_t currentTimeMs = currentTimeUs / 1000;

    if (_cmsx.isInMenu()) {
        if (scanKeys(currentTimeMs, _lastCalledMs)) {
            _cmsx.drawMenu(*_displayPort, currentTimeUs);
            _displayPort->commitTransaction();
        }
        if (currentTimeMs > _lastHeartbeatTimeMs + HEARTBEAT_INTERVAL_MS) {
            // Heart beat for external CMS display device @500ms, timeout @1000ms
            _displayPort->heartbeat();
            _lastHeartbeatTimeMs = currentTimeMs;
        }
    } else {
        // Detect menu invocation
        if (!_cockpit.isArmed() && !_cockpit.isRcModeActive(MSP_Box::BOX_STICK_COMMAND_DISABLE)) {
            const ReceiverBase::controls_pwm_t controls = _receiver.getControlsPWM();
#if defined(LIBRARY_RECEIVER_USE_ESPNOW )
            if (Cockpit::pwmIsLow(controls.yaw) && Cockpit::pwmIsHigh(controls.pitch)) {
#else
            if (Cockpit::pwmIsLow(controls.yaw) && Cockpit::pwmIsHigh(controls.pitch) && Cockpit::pwmIsMid(controls.throttle)) {
#endif
                _displayPort->beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
                _cmsx.menuOpen(*_displayPort);
                _cmsx.drawMenu(*_displayPort, currentTimeUs);
                _displayPort->commitTransaction();
                _keyDelayMs = CMSX::BUTTON_PAUSE_MS; // Tends to overshoot if BUTTON_TIME_MS used
            }
        }
    }
    // Some commands, notably flash erase, take a long time, so currentTimeMs is not an accurate value for _lastCalledMs
    _lastCalledMs = timeMs();
}

void CMS::setExternKey(CMSX::key_e externKey)
{
    if (_externKey == CMSX::KEY_NONE) {
        _externKey = externKey;
    }
}

uint16_t CMS::handleKeyWithRepeat(CMSX::key_e key, size_t repeatCount)
{
    uint16_t ret = 0;
    for (size_t ii = 0; ii < repeatCount ; ++ii) {
        ret = _cmsx.handleKey(*_displayPort, key);
    }
    return ret;
}

bool CMS::scanKeys(uint32_t currentTimeMs, uint32_t lastCalledMs) // NOLINT(readability-function-cognitive-complexity)
{
    if (_externKey != CMSX::KEY_NONE) {
        _keyDelayMs = _cmsx.handleKey(*_displayPort, _externKey);
        _externKey = CMSX::KEY_NONE;
        _displayPort->beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        return true;
    }

    const ReceiverBase::controls_pwm_t controls = _receiver.getControlsPWM();

    CMSX::key_e key = CMSX::KEY_NONE;
    if (_cockpit.isArmed() == false && Cockpit::pwmIsMid(controls.throttle) && Cockpit::pwmIsLow(controls.yaw) && Cockpit::pwmIsHigh(controls.pitch)) {
        key = CMSX::KEY_MENU;
    } else if (Cockpit::pwmIsHigh(controls.pitch)) {
        key = CMSX::KEY_UP;
    } else if (Cockpit::pwmIsLow(controls.pitch)) {
        key = CMSX::KEY_DOWN;
    } else if (Cockpit::pwmIsHigh(controls.roll)) {
        key = CMSX::KEY_RIGHT;
    } else if (Cockpit::pwmIsLow(controls.roll)) {
        key = CMSX::KEY_LEFT;
    } else if (Cockpit::pwmIsHigh(controls.yaw)) {
        key = CMSX::KEY_ESC;
    } else if (Cockpit::pwmIsLow(controls.yaw)) {
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
        _keyDelayMs -= static_cast<int32_t>(currentTimeMs - lastCalledMs);
        return false;
    }
    if (key) {
        _displayPort->beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        _keyDelayMs = handleKeyWithRepeat(key, _repeatCount);
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
    for (size_t ii = 0; ii < _deviceCount; ++ii) {
        if (displayPortSelectNext() == displayPort) {
            return true;
        }
    }
    return false;
}
