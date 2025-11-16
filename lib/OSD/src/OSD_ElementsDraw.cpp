#include "OSD.h"
#include "OSD_Elements.h"
#include "OSD_Symbols.h"

#include "AHRS_MessageQueue.h"
#include "Cockpit.h"
#include "Debug.h"
#include "FlightController.h"
#include "ReceiverBase.h"

//
// Drawing functions
//

std::array<OSD_Elements::elementDrawFnPtr, OSD_ITEM_COUNT> OSD_Elements::elementDrawFunctions {};
std::array<OSD_Elements::elementDrawFnPtr, OSD_ITEM_COUNT> OSD_Elements::elementDrawBackgroundFunctions {};


void OSD_Elements::initDrawFunctions()
{
    elementDrawFunctions.fill(nullptr);
    elementDrawBackgroundFunctions.fill(nullptr);

    elementDrawFunctions[OSD_RSSI_VALUE]        = &OSD_Elements::drawRSSI;
    elementDrawFunctions[OSD_MAIN_BATT_VOLTAGE] = &OSD_Elements::drawMainBatteryVoltage;
    elementDrawFunctions[OSD_CROSSHAIRS]        = &OSD_Elements::drawCrosshairs;
    elementDrawFunctions[OSD_ARTIFICIAL_HORIZON] = &OSD_Elements::drawArtificialHorizon,
    elementDrawFunctions[OSD_ROLL_PIDS]         = &OSD_Elements::drawPIDsRoll;
    elementDrawFunctions[OSD_PITCH_PIDS]        = &OSD_Elements::drawPIDsPitch;
    elementDrawFunctions[OSD_YAW_PIDS]          = &OSD_Elements::drawPIDsYaw;
    elementDrawFunctions[OSD_DEBUG]             = &OSD_Elements::drawDebug;
    elementDrawFunctions[OSD_PITCH_ANGLE]       = &OSD_Elements::drawAnglePitch;
    elementDrawFunctions[OSD_ROLL_ANGLE]        = &OSD_Elements::drawAngleRoll;
    elementDrawFunctions[OSD_DEBUG2]            = &OSD_Elements::drawDebug2;
    elementDrawFunctions[OSD_RC_CHANNELS]       = &OSD_Elements::drawRC_Channels;
    elementDrawBackgroundFunctions[OSD_HORIZON_SIDEBARS]    = &OSD_Elements::drawBackgroundHorizonSidebars;
};

void OSD_Elements::formatPID(char* buf, const char* label, uint8_t axis) // NOLINT(readability-non-const-parameter)
{
    const FlightController::PIDF_uint16_t pid = _flightController.getPID_MSP(axis);
    sprintf(buf, "%s %3d %3d %3d %3d %3d", label,
        pid.kp,
        pid.ki,
        pid.kd,
        pid.ks,
        pid.kk
    );
}

void OSD_Elements::drawPIDsRoll(DisplayPortBase& displayPort)
{
    (void)displayPort;
    formatPID(&_activeElement.buf[0], "PIDR", FlightController::ROLL_RATE_DPS);
}

void OSD_Elements::drawPIDsPitch(DisplayPortBase& displayPort)
{
    (void)displayPort;
    formatPID(&_activeElement.buf[0], "PIDP", FlightController::PITCH_RATE_DPS);
}

void OSD_Elements::drawPIDsYaw(DisplayPortBase& displayPort)
{
    (void)displayPort;
    formatPID(&_activeElement.buf[0], "PIDY", FlightController::YAW_RATE_DPS);
}

void OSD_Elements::drawAngleRoll(DisplayPortBase& displayPort)
{
    (void)displayPort;
#if defined(M5_UNIFIED)
    sprintf(&_activeElement.buf[0], "ro:%4d", static_cast<int>(_rollAngleDegrees));
#else
    sprintf(&_activeElement.buf[0], "ROL%4d", static_cast<int>(_rollAngleDegrees));
#endif
}

void OSD_Elements::drawAnglePitch(DisplayPortBase& displayPort)
{
    (void)displayPort;
#if defined(M5_UNIFIED)
    sprintf(&_activeElement.buf[0], "pi:%4d", static_cast<int>(_pitchAngleDegrees));
#else
    sprintf(&_activeElement.buf[0], "PIT%4d", static_cast<int>(_pitchAngleDegrees));
#endif
}

void OSD_Elements::drawRC_Channels(DisplayPortBase& displayPort) // cppcheck-suppress constParameterCallback
{
    const ReceiverBase::controls_pwm_t controlsPWM = _cockpit.getReceiver().getControlsPWM();
    switch (_rcChannel) {
    case 0:
        sprintf(&_activeElement.buf[0], "T:%5d", controlsPWM.throttle);
        _activeElement.offsetX = 0;
        _activeElement.offsetY = 0;
        break;
    case 1:
        sprintf(&_activeElement.buf[0], "R:%5d", controlsPWM.roll);
        _activeElement.offsetX = displayPort.getColumnCount()/2;
        _activeElement.offsetY = 0;
        break;
    case 2:
        sprintf(&_activeElement.buf[0], "P:%5d", controlsPWM.pitch);
        _activeElement.offsetX = displayPort.getColumnCount()/2;
        _activeElement.offsetY = 1;
        break;
    default:
        sprintf(&_activeElement.buf[0], "Y:%5d", controlsPWM.yaw);
        _activeElement.offsetX = 0;
        _activeElement.offsetY = 1;
        break;
    }

    if (++_rcChannel == ReceiverBase::STICK_COUNT) {
        _rcChannel = 0;
        _activeElement.rendered = true;
    } else {
        // rendering not complete until all 4 channels rendered
        _activeElement.rendered = false;
    }
}

void OSD_Elements::drawRSSI(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::drawMainBatteryVoltage(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::drawCrosshairs(DisplayPortBase& displayPort)
{
    (void)displayPort;
    _activeElement.buf[0] = SYM_AH_CENTER_LINE;
    _activeElement.buf[1] = SYM_AH_CENTER;
    _activeElement.buf[2] = SYM_AH_CENTER_LINE_RIGHT;
    _activeElement.buf[3] = 0;
}

void OSD_Elements::drawArtificialHorizon(DisplayPortBase& displayPort)
{
    (void)displayPort;

    enum { AH_SYMBOL_COUNT = 9 };
    enum { AH_SYMBOL_SIDE_COUNT = 4 };
    static int x = -AH_SYMBOL_SIDE_COUNT;
    // Get pitch and roll limits in tenths of degrees
    const float maxPitch = static_cast<float>(_osd.getConfig().ahMaxPitch) * 10.0F;
    const float maxRoll = static_cast<float>(_osd.getConfig().ahMaxRoll) * 10.0F;
    const float ahSign = _osd.getConfig().ahInvert ? -1.0F : 1.0F;
    const float rollAngle = std::clamp(_flightController.getRollAngleDegreesRaw() * ahSign, -maxRoll, maxRoll);
    float pitchAngle = std::clamp(_flightController.getRollAngleDegreesRaw() * ahSign, -maxPitch, maxPitch);
    // Convert pitchAngle to y compensation value
    // (maxPitch / 25) divisor matches previous settings of fixed divisor of 8 and fixed max AHI pitch angle of 20.0 degrees
    if (maxPitch > 0.0F) {
        pitchAngle = ((pitchAngle * 25.0F) / maxPitch);
    }
    pitchAngle -= 4 * AH_SYMBOL_COUNT + 5;

    const int y = ((-static_cast<int>(rollAngle) * x) / 64) - static_cast<int>(pitchAngle);
    if (y >= 0 && y <= 81) {
        _activeElement.offsetX = static_cast<uint8_t>(x);
        _activeElement.offsetY = static_cast<uint8_t>(y / AH_SYMBOL_COUNT);

        sprintf(&_activeElement.buf[0], "%c", (SYM_AH_BAR9_0 + (y % AH_SYMBOL_COUNT)));
    } else {
        _activeElement.drawElement = false;  // element does not need to be rendered
    }

    if (x == AH_SYMBOL_SIDE_COUNT) {
        // Rendering is complete, so prepare to start again
        x = -AH_SYMBOL_SIDE_COUNT;
    } else {
        // Rendering not yet complete
        _activeElement.rendered = false;
        ++x;
    }
}

void OSD_Elements::drawBackgroundHorizonSidebars(DisplayPortBase& displayPort)
{
    // Draw AH sides
    const int8_t width = AH_SIDEBAR_WIDTH_POS;
    const int8_t height = AH_SIDEBAR_HEIGHT_POS;

    if (_sideBarRenderLevel) {
        // AH level indicators
        displayPort.writeChar(_activeElement.posX - width + 1, _activeElement.posY, DisplayPortBase::SEVERITY_NORMAL, SYM_AH_LEFT);
        displayPort.writeChar(_activeElement.posX + width - 1, _activeElement.posY, DisplayPortBase::SEVERITY_NORMAL, SYM_AH_RIGHT);
        _sideBarRenderLevel = false;
    } else {
        displayPort.writeChar(_activeElement.posX - width, _activeElement.posY + static_cast<uint8_t>(_sidbarPosY), DisplayPortBase::SEVERITY_NORMAL, SYM_AH_DECORATION);
        displayPort.writeChar(_activeElement.posX + width, _activeElement.posY + static_cast<uint8_t>(_sidbarPosY), DisplayPortBase::SEVERITY_NORMAL, SYM_AH_DECORATION);
        if (_sidbarPosY == height) {
            // Rendering is complete, so prepare to start again
            _sidbarPosY = -height;
            // On next pass render the level markers
            _sideBarRenderLevel = true;
        } else {
            ++_sidbarPosY;
        }
        // Rendering not yet complete
        _activeElement.rendered = false;
    }

    _activeElement.drawElement = false;  // element already drawn
}

void OSD_Elements::drawDebug(DisplayPortBase& displayPort)
{
    (void)displayPort;
    sprintf(&_activeElement.buf[0], "DBG %5d %5d %5d %5d", _debug.get(0), _debug.get(1),_debug.get(2),_debug.get(3));
}

void OSD_Elements::drawDebug2(DisplayPortBase& displayPort)
{
    (void)displayPort;
    sprintf(&_activeElement.buf[0], "DBG %5d %5d %5d %5d", _debug.get(4), _debug.get(5),_debug.get(6),_debug.get(7));
}
