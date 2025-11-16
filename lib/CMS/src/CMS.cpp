#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "DisplayPortBase.h"

#include <MSP_Box.h>
#include <ReceiverBase.h>


CMS::CMS(DisplayPortBase* displayPort, const ReceiverBase& receiver, Cockpit& cockpit, IMU_Filters& imuFilters, OSD* osd) :
    _displayPort(displayPort),
    _cmsx(*this),
    _receiver(receiver),
    _cockpit(cockpit),
    _imuFilters(imuFilters),
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
        _displayPort->beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        _rcDelayMs = scanKeys(currentTimeMs, _lastCalledMs, _rcDelayMs);
        _cmsx.drawMenu(*_displayPort, currentTimeUs);
        if (currentTimeMs > _lastHeartbeatTimeMs + HEARTBEAT_INTERVAL_MS) {
            // Heart beat for external CMS display device @500ms, timeout @1000ms
            _displayPort->heartbeat();
            _lastHeartbeatTimeMs = currentTimeMs;
        }
        _displayPort->commitTransaction();
    } else {
        // Detect menu invocation
        if (!_cockpit.isArmed() && !_cockpit.isRcModeActive(MSP_Box::BOX_STICK_COMMAND_DISABLE)) {
            const ReceiverBase::controls_pwm_t controls = _receiver.getControlsPWM();
            if (pwmIsMid(controls.throttle) && pwmIsLow(controls.yaw) && pwmIsHigh(controls.pitch)) {
                _cmsx.menuOpen(*_displayPort);
                _rcDelayMs = CMSX::BUTTON_PAUSE_MS; // Tends to overshoot if BUTTON_TIME_MS used
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

uint32_t CMS::scanKeys(uint32_t currentTimeMs, uint32_t lastCalledMs, uint32_t rcDelayMs)
{
    CMSX::key_e key = CMSX::KEY_NONE;

    const ReceiverBase::controls_pwm_t controls = _receiver.getControlsPWM();

    if (_externKey != CMSX::KEY_NONE) {
        rcDelayMs = _cmsx.handleKey(*_displayPort, _externKey);
        _externKey = CMSX::KEY_NONE;
    } else {
        if (_cockpit.isArmed() == false && pwmIsMid(controls.throttle) && pwmIsLow(controls.yaw) && pwmIsHigh(controls.pitch)) {
            key = CMSX::KEY_MENU;
        } else if (pwmIsHigh(controls.pitch)) {
            key = CMSX::KEY_UP;
        } else if (pwmIsLow(controls.pitch)) {
            key = CMSX::KEY_DOWN;
        } else if (pwmIsHigh(controls.roll)) {
            key = CMSX::KEY_RIGHT;
        } else if (pwmIsLow(controls.roll)) {
            key = CMSX::KEY_LEFT;
        } else if (pwmIsHigh(controls.yaw)) {
            key = CMSX::KEY_ESC;
        } else if (pwmIsLow(controls.yaw)) {
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

        if (rcDelayMs > 0) {
            rcDelayMs -= (currentTimeMs - lastCalledMs);
        } else if (key) {
            rcDelayMs = handleKeyWithRepeat(key, _repeatCount);
            // Key repeat effect is implemented in two phases.
            // First phase is to decrease rcDelayMs reciprocal to hold time.
            // When rcDelayMs reached a certain limit (scheduling interval),
            // repeat rate will not raise anymore, so we call key handler
            // multiple times (repeatCount).
            //
            // XXX Caveat: Most constants are adjusted pragmatically.
            // XXX Rewrite this someday, so it uses actual hold time instead
            // of holdCount, which depends on the scheduling interval.
            if (((key == CMSX::KEY_LEFT) || (key == CMSX::KEY_RIGHT)) && (_holdCount > 20)) {
                // Decrease rcDelayMs reciprocally
                rcDelayMs /= (_holdCount - 20);
                // When we reach the scheduling limit,
                if (rcDelayMs <= 50) {
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
        }
    }
    return rcDelayMs;
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
