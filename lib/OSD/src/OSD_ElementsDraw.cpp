#include "OSD.h"
#include "OSD_Elements.h"
#include "OSD_Symbols.h"

#include "AHRS_MessageQueue.h"
#include "Debug.h"
#include "FlightController.h"

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

    elementDrawBackgroundFunctions[OSD_HORIZON_SIDEBARS]    = &OSD_Elements::drawBackgroundHorizonSidebars;
};

void OSD_Elements::formatPID(char * buf, const char * label, uint8_t axis) // NOLINT(readability-non-const-parameter)
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

bool OSD_Elements::drawPIDsRoll(DisplayPortBase& displayPort, element_t&element)
{
    (void)displayPort;
    formatPID(&element.buf[0], "PIDR", FlightController::ROLL_RATE_DPS);
    return true;
}

bool OSD_Elements::drawPIDsPitch(DisplayPortBase& displayPort, element_t& element)
{
    (void)displayPort;
    formatPID(&element.buf[0], "PIDP", FlightController::PITCH_RATE_DPS);
    return true;
}

bool OSD_Elements::drawPIDsYaw(DisplayPortBase& displayPort, element_t&element)
{
    (void)displayPort;
    formatPID(&element.buf[0], "PIDY", FlightController::YAW_RATE_DPS);
    return true;
}

bool OSD_Elements::drawAngleRoll(DisplayPortBase& displayPort, element_t&element)
{
    (void)displayPort;
    AHRS::ahrs_data_t ahrsData {};
    _ahrsMessageQueue.PEEK_AHRS_DATA(ahrsData);
    const float rollAngleDegrees = ahrsData.orientation.calculateRollDegrees();
    sprintf(&element.buf[0], "ROL %3.1f", rollAngleDegrees);
    return true;
}

bool OSD_Elements::drawAnglePitch(DisplayPortBase& displayPort, element_t&element)
{
    (void)displayPort;
    AHRS::ahrs_data_t ahrsData {};
    _ahrsMessageQueue.PEEK_AHRS_DATA(ahrsData);
    const float pitchAngleDegrees = ahrsData.orientation.calculatePitchDegrees();
    sprintf(&element.buf[0], "PIT %3.1f", pitchAngleDegrees);
    return true;
}

bool OSD_Elements::drawRSSI(DisplayPortBase& displayPort, element_t&element)
{
    (void)element;
    (void)displayPort;
    return true;
}

bool OSD_Elements::drawMainBatteryVoltage(DisplayPortBase& displayPort, element_t&element)
{
    (void)displayPort;
    (void)element;
    return true;
}

bool OSD_Elements::drawCrosshairs(DisplayPortBase& displayPort, element_t&element)
{
    (void)displayPort;
    element.buf[0] = '-';//SYM_AH_CENTER_LINE;
    element.buf[1] = '+';//SYM_AH_CENTER;
    element.buf[2] = '-';//SYM_AH_CENTER_LINE_RIGHT;
    element.buf[3] = 0;
    return true;
}

bool OSD_Elements::drawArtificialHorizon(DisplayPortBase& displayPort, element_t& element)
{
    (void)displayPort;

    bool ret = true;

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
        element.offsetX = static_cast<uint8_t>(x);
        element.offsetY = static_cast<uint8_t>(y / AH_SYMBOL_COUNT);

        sprintf(&element.buf[0], "%c", (SYM_AH_BAR9_0 + (y % AH_SYMBOL_COUNT)));
    } else {
        ret = false;  // element does not need to be rendered
    }

    if (x == AH_SYMBOL_SIDE_COUNT) {
        // Rendering is complete, so prepare to start again
        x = -AH_SYMBOL_SIDE_COUNT;
    } else {
        // Rendering not yet complete
        element.rendered = false;
        ++x;
    }

    return ret;
}

bool OSD_Elements::drawBackgroundHorizonSidebars(DisplayPortBase& displayPort, element_t& element)
{
    // Draw AH sides
    const int8_t width = AH_SIDEBAR_WIDTH_POS;
    const int8_t height = AH_SIDEBAR_HEIGHT_POS;

    if (_sideBarRenderLevel) {
        // AH level indicators
        displayPort.writeChar(element.posX - width + 1, element.posY, DisplayPortBase::SEVERITY_NORMAL, SYM_AH_LEFT);
        displayPort.writeChar(element.posX + width - 1, element.posY, DisplayPortBase::SEVERITY_NORMAL, SYM_AH_RIGHT);
        _sideBarRenderLevel = false;
    } else {
        displayPort.writeChar(element.posX - width, element.posY + static_cast<uint8_t>(_sidbarPosY), DisplayPortBase::SEVERITY_NORMAL, SYM_AH_DECORATION);
        displayPort.writeChar(element.posX + width, element.posY + static_cast<uint8_t>(_sidbarPosY), DisplayPortBase::SEVERITY_NORMAL, SYM_AH_DECORATION);
        if (_sidbarPosY == height) {
            // Rendering is complete, so prepare to start again
            _sidbarPosY = -height;
            // On next pass render the level markers
            _sideBarRenderLevel = true;
        } else {
            ++_sidbarPosY;
        }
        // Rendering not yet complete
        element.rendered = false;
    }

    return false;  // element already drawn
}

bool OSD_Elements::drawDebug(DisplayPortBase& displayPort, element_t& element)
{
    (void)displayPort;
    sprintf(&element.buf[0], "DBG %5d %5d %5d %5d", _debug.get(0), _debug.get(1),_debug.get(2),_debug.get(3));
    return true;
}

bool OSD_Elements::drawDebug2(DisplayPortBase& displayPort, element_t& element)
{
    (void)displayPort;
    sprintf(&element.buf[0], "DBG %5d %5d %5d %5d", _debug.get(4), _debug.get(5),_debug.get(6),_debug.get(7));
    return true;
}
