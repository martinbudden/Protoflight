#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "DisplayPortBase.h"

#include <MSP_Box.h>
#include <ReceiverBase.h>


CMS::CMS(OSD& osd, const ReceiverBase& receiver, const FlightController& flightController, Cockpit& cockpit) :
    _osd(osd),
    _receiver(receiver),
    _flightController(flightController),
    _cockpit(cockpit)
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

void CMS::updateCMS(uint32_t currentTimeUs, uint32_t timeMicrosecondsDelta) // NOLINT(readability-function-cognitive-complexity)
{
    (void)timeMicrosecondsDelta;

    if (_cockpit.isRcModeActive(MSP_Box::BOX_PARALYZE)) {
        return;
    }

    const uint32_t currentTimeMs = currentTimeUs / 1000;

    if (_inMenu) {
        _displayPort->beginTransaction(DISPLAY_TRANSACTION_OPT_RESET_DRAWING);
        _rcDelayMs = static_cast<int32_t>(scanKeys(currentTimeMs, _lastCalledMs, _rcDelayMs));
        drawMenu(currentTimeUs);
        if (currentTimeMs > _lastHeartbeatTimeMs + 500) {
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
                menuOpen();
                _rcDelayMs = BUTTON_PAUSE_MS; // Tends to overshoot if BUTTON_TIME_MS used
            }
        }
    }

    // Some commands, notably flash erase, take a long time, so currentTimeMs is not an accurate value for _lastCalledMs
    _lastCalledMs = timeMs();
}

void CMS::setExternKey(key_e externKey)
{
    if (_externKey == KEY_NONE) {
        _externKey = externKey;
    }
}

uint32_t CMS::handleKey(key_e key)
{
    const uint32_t ret = BUTTON_TIME_MS;

    if (key == KEY_MENU) {
        menuOpen();
        return BUTTON_PAUSE_MS;
    }
    return ret;
}

void CMS::drawMenu(uint32_t currentTimeUs)
{
    (void)currentTimeUs;
}

void CMS::menuOpen()
{
    const CMSX::menu_t* startMenu {};
    if (_inMenu) {
        // Switch display
        DisplayPortBase* nextDisplayPort = displayPortSelectNext();
        startMenu = CMSX::currentCtx.menu;
        if (nextDisplayPort == _displayPort) {
            return;
        }
        // DisplayPort has been changed.
        CMSX::currentCtx.cursorRow = CMSX::cursorAbsolute();
        _displayPort->setBackgroundType(DisplayPortBase::BACKGROUND_TRANSPARENT); // reset previous displayPort to transparent
        _displayPort->release();
        _displayPort = nextDisplayPort;
    } else {
        //_displayPort = cmsDisplayPortSelectCurrent();
        if (!_displayPort) {
            return;
        }
        _inMenu = true;
        CMSX::currentCtx = { nullptr, 0, 0 };
        startMenu = &CMSX::menuMain;
        CMSX::menuStackIndex = 0;
        //setArmingDisabled(ARMING_DISABLED_CMS_MENU);
        _displayPort->layerSelect(DisplayPortBase::LAYER_FOREGROUND);
    }
    //!!other stuff here
    (void)startMenu;
    //CMSX::menuChange(startMenu, *this, *_displayPort, nullptr);
    CMSX::menuChange(*this, *_displayPort, nullptr);
}

uint32_t CMS::scanKeys(uint32_t currentTimeMs, uint32_t lastCalledMs, uint32_t rcDelayMs)
{
    (void)currentTimeMs;
    (void)lastCalledMs;
    (void)rcDelayMs;
    return 0;
}

void CMS::inhibitSaveMenu()
{
    _saveMenuInhibited = true;
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

    return _displayPorts[_currentDeviceIndex];
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
