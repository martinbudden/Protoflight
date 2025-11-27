#include "Cockpit.h"
#include "DisplayPortBase.h"
#include "FlightController.h"
#include "OSD.h"
#include "OSD_Elements.h"
#include "OSD_Symbols.h"

#include <AHRS_MessageQueue.h>
#include <Debug.h>
#include <ReceiverBase.h>

//
// Drawing functions
//

std::array<OSD_Elements::elementDrawFnPtr, OSD_ITEM_COUNT> OSD_Elements::DrawFunctions {};
std::array<OSD_Elements::elementDrawFnPtr, OSD_ITEM_COUNT> OSD_Elements::DrawBackgroundFunctions {};


void OSD_Elements::initDrawFunctions()
{
    DrawFunctions.fill(nullptr);
    DrawBackgroundFunctions.fill(nullptr);

    DrawFunctions[OSD_RSSI_VALUE]       = &OSD_Elements::draw_RSSI_VALUE;
    DrawFunctions[OSD_MAIN_BATTERY_VOLTAGE]= &OSD_Elements::draw_MAIN_BATTERY_VOLTAGE;
    DrawFunctions[OSD_CROSSHAIRS]       = &OSD_Elements::draw_CROSSHAIRS;
    DrawFunctions[OSD_ARTIFICIAL_HORIZON] = &OSD_Elements::draw_ARTIFICIAL_HORIZON;
    DrawFunctions[OSD_HORIZON_SIDEBARS] = nullptr; // only has background

    DrawFunctions[OSD_ITEM_TIMER_1]     = &OSD_Elements::draw_ITEM_TIMER;
    DrawFunctions[OSD_ITEM_TIMER_2]     = &OSD_Elements::draw_ITEM_TIMER;
    DrawFunctions[OSD_FLYMODE]          = &OSD_Elements::draw_FLYMODE;
    DrawFunctions[OSD_CRAFT_NAME]       = nullptr; // only has background
    DrawFunctions[OSD_THROTTLE_POS]     = &OSD_Elements::draw_THROTTLE_POS;
    DrawFunctions[OSD_VTX_CHANNEL]      = &OSD_Elements::draw_VTX_CHANNEL;
    DrawFunctions[OSD_CURRENT_DRAW]     = &OSD_Elements::draw_CURRENT_DRAW;
    DrawFunctions[OSD_MAH_DRAWN]        = &OSD_Elements::draw_MAH_DRAWN;
#if defined(USE_GPS)
    DrawFunctions[OSD_GPS_SPEED]        = &OSD_Elements::draw_GPS_SPEED;
    DrawFunctions[OSD_GPS_SATS]         = &OSD_Elements::draw_GPS_SATS;
#endif
#if defined(USE_BAROMETER) || defined(USE_GPS)
    DrawFunctions[OSD_ALTITUDE]         = &OSD_Elements::draw_ALTITUDE;
#endif
    DrawFunctions[OSD_ROLL_PIDS]        = &OSD_Elements::draw_ROLL_PIDS;
    DrawFunctions[OSD_PITCH_PIDS]       = &OSD_Elements::draw_PITCH_PIDS;
    DrawFunctions[OSD_YAW_PIDS]         = &OSD_Elements::draw_YAW_PIDS;
    DrawFunctions[OSD_POWER]            = &OSD_Elements::draw_POWER;
    DrawFunctions[OSD_PID_RATE_PROFILE] = &OSD_Elements::draw_PID_RATE_PROFILE;
    DrawFunctions[OSD_WARNINGS]         = &OSD_Elements::draw_WARNINGS;
    DrawFunctions[OSD_AVG_CELL_VOLTAGE] = &OSD_Elements::draw_AVG_CELL_VOLTAGE;
#if defined(USE_GPS)
    DrawFunctions[OSD_GPS_LON]          = &OSD_Elements::draw_GPS_LAT_LONG;
    DrawFunctions[OSD_GPS_LAT]          = &OSD_Elements::draw_GPS_LAT_LONG;
#endif
    DrawFunctions[OSD_DEBUG]            = &OSD_Elements::draw_DEBUG;
    DrawFunctions[OSD_PITCH_ANGLE]      = &OSD_Elements::draw_PITCH_ANGLE;
    DrawFunctions[OSD_ROLL_ANGLE]       = &OSD_Elements::draw_ROLL_ANGLE;
    DrawFunctions[OSD_MAIN_BATTERY_USAGE] = &OSD_Elements::draw_MAIN_BATTERY_USAGE;
    DrawFunctions[OSD_DISARMED]         = &OSD_Elements::draw_DISARMED;
#if defined(USE_GPS)
    DrawFunctions[OSD_HOME_DIRECTION]   = &OSD_Elements::draw_HOME_DIRECTION;
    DrawFunctions[OSD_HOME_DISTANCE]    = &OSD_Elements::draw_HOME_DISTANCE;
#endif
    DrawFunctions[OSD_NUMERICAL_HEADING]= &OSD_Elements::draw_NUMERICAL_HEADING;
    DrawFunctions[OSD_NUMERICAL_VARIO]  = &OSD_Elements::draw_NUMERICAL_VARIO;
    DrawFunctions[OSD_COMPASS_BAR]      = &OSD_Elements::draw_COMPASS_BAR;
#if defined(USE_DSHOT)
    DrawFunctions[OSD_ESC_TEMPERATURE]  = &OSD_Elements::draw_ESC_TEMPERATURE;
    DrawFunctions[OSD_ESC_RPM]          = &OSD_Elements::draw_ESC_RPM;
#endif
    DrawFunctions[OSD_REMAINING_TIME_ESTIMATE] = &OSD_Elements::draw_REMAINING_TIME_ESTIMATE;
    DrawFunctions[OSD_RTC_DATETIME]     = &OSD_Elements::draw_RTC_DATETIME;
    DrawFunctions[OSD_ADJUSTMENT_RANGE] = &OSD_Elements::draw_ADJUSTMENT_RANGE;
    DrawFunctions[OSD_CORE_TEMPERATURE] = &OSD_Elements::draw_CORE_TEMPERATURE;
    DrawFunctions[OSD_ANTI_GRAVITY]     = &OSD_Elements::draw_ANTI_GRAVITY;
    DrawFunctions[OSD_G_FORCE]          = &OSD_Elements::draw_G_FORCE;
    DrawFunctions[OSD_MOTOR_DIAGNOSTICS] = &OSD_Elements::draw_MOTOR_DIAGNOSTICS;
    DrawFunctions[OSD_LOG_STATUS]       = &OSD_Elements::draw_LOG_STATUS;
    DrawFunctions[OSD_FLIP_ARROW]       = &OSD_Elements::draw_FLIP_ARROW;
    DrawFunctions[OSD_LINK_QUALITY]     = &OSD_Elements::draw_LINK_QUALITY;
#if defined(USE_GPS)
    DrawFunctions[OSD_FLIGHT_DISTANCE]  = &OSD_Elements::draw_FLIGHT_DISTANCE;
#endif
    DrawFunctions[OSD_STICK_OVERLAY_LEFT] = &OSD_Elements::draw_STICK_OVERLAY;
    DrawFunctions[OSD_STICK_OVERLAY_RIGHT] = &OSD_Elements::draw_STICK_OVERLAY;
    DrawFunctions[OSD_PILOT_NAME]       = &OSD_Elements::draw_PILOT_NAME;
#if defined(USE_DSHOT)
    DrawFunctions[OSD_ESC_RPM_FREQUENCY]= &OSD_Elements::draw_ESC_RPM_FREQUENCY;
#endif
#if defined(USE_PROFILE_NAMES)
    DrawFunctions[OSD_RATE_PROFILE_NAME]= &OSD_Elements::draw_RATE_PROFILE_NAME;
    DrawFunctions[OSD_PID_PROFILE_NAME] = &OSD_Elements::draw_PID_PROFILE_NAME;
#endif
#if defined(USE_OSD_PROFILES)
    DrawFunctions[OSD_PROFILE_NAME]     = &OSD_Elements::draw_PROFILE_NAME;
#endif
    DrawFunctions[OSD_RSSI_DBM_VALUE]   = &OSD_Elements::draw_RSSI_DBM_VALUE;
    DrawFunctions[OSD_RC_CHANNELS]      = &OSD_Elements::draw_RC_CHANNELS;
    DrawFunctions[OSD_CAMERA_FRAME]     = nullptr;  // only has background. Should be added first so it's the lowest layer and doesn't cover other elements
#if defined(USE_GPS)
    DrawFunctions[OSD_EFFICIENCY]       = &OSD_Elements::draw_EFFICIENCY;
#endif
    DrawFunctions[OSD_TOTAL_FLIGHTS]    = &OSD_Elements::draw_TOTAL_FLIGHTS;
    DrawFunctions[OSD_UP_DOWN_REFERENCE]= &OSD_Elements::draw_UP_DOWN_REFERENCE;
    DrawFunctions[OSD_TX_UPLINK_POWER]  = &OSD_Elements::draw_TX_UPLINK_POWER;
    DrawFunctions[OSD_WATT_HOURS_DRAWN] = &OSD_Elements::draw_WATT_HOURS_DRAWN;
    DrawFunctions[OSD_AUX_VALUE]        = &OSD_Elements::draw_AUX_VALUE;
    DrawFunctions[OSD_READY_MODE]       = &OSD_Elements::draw_READY_MODE;
    DrawFunctions[OSD_RSNR_VALUE]       = &OSD_Elements::draw_RSNR_VALUE;
#if defined(USE_MSP_DISPLAYPORT)
    DrawFunctions[OSD_SYS_GOGGLE_VOLTAGE] = &OSD_Elements::draw_SYS;
    DrawFunctions[OSD_SYS_VTX_VOLTAGE]   = &OSD_Elements::draw_SYS;
    DrawFunctions[OSD_SYS_BITRATE]      = &OSD_Elements::draw_SYS;
    DrawFunctions[OSD_SYS_DELAY]        = &OSD_Elements::draw_SYS;
    DrawFunctions[OSD_SYS_DISTANCE]     = &OSD_Elements::draw_SYS;
    DrawFunctions[OSD_SYS_LQ]           = &OSD_Elements::draw_SYS;
    DrawFunctions[OSD_SYS_GOGGLE_DVR]   = &OSD_Elements::draw_SYS;
    DrawFunctions[OSD_SYS_VTX_DVR]      = &OSD_Elements::draw_SYS;
    DrawFunctions[OSD_SYS_WARNINGS]     = &OSD_Elements::draw_SYS;
    DrawFunctions[OSD_SYS_VTX_TEMP]     = &OSD_Elements::draw_SYS;
    DrawFunctions[OSD_SYS_FAN_SPEED]    = &OSD_Elements::draw_SYS;
#endif
#if defined(USE_GPS_LAP_TIMER)
    DrawFunctions[OSD_GPS_LAP_TIME_CURRENT] = &OSD_Elements::draw_GPS_LAP_TIME_CURRENT;
    DrawFunctions[OSD_GPS_LAP_TIME_PREVIOUS] = &OSD_Elements::draw_GPS_LAP_TIME_PREVIOUS;
    DrawFunctions[OSD_GPS_LAP_TIME_BEST3] = &OSD_Elements::draw_GPS_LAP_TIME_BEST3;
#endif
    DrawFunctions[OSD_DEBUG2]           = &OSD_Elements::draw_DEBUG2;
    DrawFunctions[OSD_CUSTOM_MSG0]      = &OSD_Elements::draw_CUSTOM_MSG;
    DrawFunctions[OSD_CUSTOM_MSG1]      = &OSD_Elements::draw_CUSTOM_MSG;
    DrawFunctions[OSD_CUSTOM_MSG2]      = &OSD_Elements::draw_CUSTOM_MSG;
    DrawFunctions[OSD_CUSTOM_MSG3]      = &OSD_Elements::draw_CUSTOM_MSG;
#if defined(USE_RANGEFINDER)
    DrawFunctions[OSD_LIDAR_DISTANCE]   = &OSD_Elements::draw_LIDAR_DISTANCE;
#endif

    DrawBackgroundFunctions[OSD_HORIZON_SIDEBARS]   = &OSD_Elements::drawBackground_HORIZON_SIDEBARS;
    DrawBackgroundFunctions[OSD_CRAFT_NAME]         = &OSD_Elements::drawBackground_CRAFT_NAME;
    DrawBackgroundFunctions[OSD_STICK_OVERLAY_LEFT] = &OSD_Elements::drawBackground_STICK_OVERLAY;
    DrawBackgroundFunctions[OSD_STICK_OVERLAY_RIGHT]= &OSD_Elements::drawBackground_STICK_OVERLAY;
    DrawBackgroundFunctions[OSD_PILOT_NAME]         = &OSD_Elements::drawBackground_PILOT_NAME;
    DrawBackgroundFunctions[OSD_CAMERA_FRAME]       = &OSD_Elements::drawBackground_CAMERA_FRAME;
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

void OSD_Elements::drawBackground_HORIZON_SIDEBARS(DisplayPortBase& displayPort)
{
    // Draw AH sides
    const int8_t width = AH_SIDEBAR_WIDTH_POS;
    const int8_t height = AH_SIDEBAR_HEIGHT_POS;

    if (_HORIZON_SIDEBARS_RenderLevel) {
        // AH level indicators
        displayPort.writeChar(_activeElement.posX - width + 1, _activeElement.posY, DisplayPortBase::SEVERITY_NORMAL, SYM_AH_LEFT);
        displayPort.writeChar(_activeElement.posX + width - 1, _activeElement.posY, DisplayPortBase::SEVERITY_NORMAL, SYM_AH_RIGHT);
        _HORIZON_SIDEBARS_RenderLevel = false;
    } else {
        displayPort.writeChar(_activeElement.posX - width, _activeElement.posY + static_cast<uint8_t>(_HORIZON_SIDEBARS_PosY), DisplayPortBase::SEVERITY_NORMAL, SYM_AH_DECORATION);
        displayPort.writeChar(_activeElement.posX + width, _activeElement.posY + static_cast<uint8_t>(_HORIZON_SIDEBARS_PosY), DisplayPortBase::SEVERITY_NORMAL, SYM_AH_DECORATION);
        if (_HORIZON_SIDEBARS_PosY == height) {
            // Rendering is complete, so prepare to start again
            _HORIZON_SIDEBARS_PosY = -height;
            // On next pass render the level markers
            _HORIZON_SIDEBARS_RenderLevel = true;
        } else {
            ++_HORIZON_SIDEBARS_PosY;
        }
        // Rendering not yet complete
        _activeElement.rendered = false;
    }
    _activeElement.drawElement = false;  // element already drawn
}

void OSD_Elements::drawBackground_CRAFT_NAME(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::drawBackground_STICK_OVERLAY(DisplayPortBase& displayPort)
{
    (void)displayPort;

    if (_STICK_OVERLAY_RenderPhase == VERTICAL) {
        sprintf(&_activeElement.buf[0], "%c", SYM_STICK_OVERLAY_VERTICAL);
        _activeElement.offsetX = ((STICK_OVERLAY_WIDTH - 1) / 2);
        _activeElement.offsetY = _STICK_OVERLAY_Y;

        ++_STICK_OVERLAY_Y;

        if (_STICK_OVERLAY_Y == (STICK_OVERLAY_HEIGHT - 1) / 2) {
            // Skip over horizontal
            ++_STICK_OVERLAY_Y;
        }

        if (_STICK_OVERLAY_Y == STICK_OVERLAY_HEIGHT) {
            _STICK_OVERLAY_Y = 0;
            _STICK_OVERLAY_RenderPhase = HORIZONTAL;
        }

        _activeElement.rendered = false;
    } else {
        for (uint8_t i = 0; i < STICK_OVERLAY_WIDTH; i++) {
            _activeElement.buf[i] = SYM_STICK_OVERLAY_HORIZONTAL;
        }
        _activeElement.buf[((STICK_OVERLAY_WIDTH - 1) / 2)] = SYM_STICK_OVERLAY_CENTER;
        _activeElement.buf[STICK_OVERLAY_WIDTH] = 0;  // string terminator

        _activeElement.offsetY = ((STICK_OVERLAY_HEIGHT - 1) / 2);

        _STICK_OVERLAY_RenderPhase = VERTICAL;
    }
}

void OSD_Elements::drawBackground_PILOT_NAME(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::drawBackground_CAMERA_FRAME(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_RSSI_VALUE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_MAIN_BATTERY_VOLTAGE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_CROSSHAIRS(DisplayPortBase& displayPort)
{
    (void)displayPort;
    _activeElement.buf[0] = SYM_AH_CENTER_LINE;
    _activeElement.buf[1] = SYM_AH_CENTER;
    _activeElement.buf[2] = SYM_AH_CENTER_LINE_RIGHT;
    _activeElement.buf[3] = 0;
}

void OSD_Elements::draw_ARTIFICIAL_HORIZON(DisplayPortBase& displayPort)
{
    (void)displayPort;

    enum { AH_SYMBOL_COUNT = 9 };
    enum { AH_SYMBOL_SIDE_COUNT = 4 };
    static int x = -AH_SYMBOL_SIDE_COUNT;
    // Get pitch and roll limits in tenths of degrees
    const float maxPitch = static_cast<float>(_osd.getConfig().ahMaxPitch) * 10.0F;
    const float maxRoll = static_cast<float>(_osd.getConfig().ahMaxRoll) * 10.0F;
    const float ahSign = _osd.getConfig().ahInvert ? -1.0F : 1.0F;
    const float rollAngle = std::clamp(_rollAngleDegrees * ahSign, -maxRoll, maxRoll);
    float pitchAngle = std::clamp(_pitchAngleDegrees * ahSign, -maxPitch, maxPitch);
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
#if defined(M5_UNIFIED)
        _activeElement.buf[0] = '-';
        _activeElement.buf[1] = 0;
#else
        sprintf(&_activeElement.buf[0], "%c", (SYM_AH_BAR9_0 + (y % AH_SYMBOL_COUNT)));
#endif
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

void OSD_Elements::draw_ITEM_TIMER(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_FLYMODE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_THROTTLE_POS(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_VTX_CHANNEL(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_CURRENT_DRAW(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_MAH_DRAWN(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_GPS_SPEED(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_GPS_SATS(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_ALTITUDE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_ROLL_PIDS(DisplayPortBase& displayPort)
{
    (void)displayPort;
    formatPID(&_activeElement.buf[0], "PIDR", FlightController::ROLL_RATE_DPS);
}

void OSD_Elements::draw_PITCH_PIDS(DisplayPortBase& displayPort)
{
    (void)displayPort;
    formatPID(&_activeElement.buf[0], "PIDP", FlightController::PITCH_RATE_DPS);
}

void OSD_Elements::draw_YAW_PIDS(DisplayPortBase& displayPort)
{
    (void)displayPort;
    formatPID(&_activeElement.buf[0], "PIDY", FlightController::YAW_RATE_DPS);
}

void OSD_Elements::draw_POWER(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_PID_RATE_PROFILE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_WARNINGS(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_AVG_CELL_VOLTAGE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_GPS_LAT_LONG(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_DEBUG(DisplayPortBase& displayPort)
{
    (void)displayPort;
    sprintf(&_activeElement.buf[0], "DBG %5d %5d %5d %5d", _debug.get(0), _debug.get(1),_debug.get(2),_debug.get(3));
}

void OSD_Elements::draw_DEBUG2(DisplayPortBase& displayPort)
{
    (void)displayPort;
    sprintf(&_activeElement.buf[0], "DBG %5d %5d %5d %5d", _debug.get(4), _debug.get(5),_debug.get(6),_debug.get(7));
}

void OSD_Elements::draw_PITCH_ANGLE(DisplayPortBase& displayPort)
{
    (void)displayPort;
#if defined(M5_UNIFIED)
    sprintf(&_activeElement.buf[0], "p:%4d", static_cast<int>(_pitchAngleDegrees));
#else
    sprintf(&_activeElement.buf[0], "PIT%4d", static_cast<int>(_pitchAngleDegrees));
#endif
}

void OSD_Elements::draw_ROLL_ANGLE(DisplayPortBase& displayPort)
{
    (void)displayPort;
#if defined(M5_UNIFIED)
    sprintf(&_activeElement.buf[0], "r:%4d", static_cast<int>(_rollAngleDegrees));
#else
    sprintf(&_activeElement.buf[0], "ROL%4d", static_cast<int>(_rollAngleDegrees));
#endif
}

void OSD_Elements::draw_MAIN_BATTERY_USAGE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_DISARMED(DisplayPortBase& displayPort)
{
    (void)displayPort;
    sprintf(&_activeElement.buf[0], _cockpit.isArmed()? "ARMED" : "DISARMED");
}

void OSD_Elements::draw_HOME_DIRECTION(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_HOME_DISTANCE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_NUMERICAL_HEADING(DisplayPortBase& displayPort)
{
    (void)displayPort;
#if defined(M5_UNIFIED)
    sprintf(&_activeElement.buf[0], "y:%4d", static_cast<int>(_yawAngleDegrees));
#else
    sprintf(&_activeElement.buf[0], "PIT%4d", static_cast<int>(_yawAngleDegrees));
#endif
}

void OSD_Elements::draw_NUMERICAL_VARIO(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_COMPASS_BAR(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_ESC_TEMPERATURE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_ESC_RPM(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_REMAINING_TIME_ESTIMATE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_RTC_DATETIME(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_ADJUSTMENT_RANGE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_CORE_TEMPERATURE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_ANTI_GRAVITY(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_G_FORCE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_MOTOR_DIAGNOSTICS(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_LOG_STATUS(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_FLIP_ARROW(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_LINK_QUALITY(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_FLIGHT_DISTANCE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_STICK_OVERLAY(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_PILOT_NAME(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_ESC_RPM_FREQUENCY(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_RATE_PROFILE_NAME(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_PID_PROFILE_NAME(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_PROFILE_NAME(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_RSSI_DBM_VALUE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_RC_CHANNELS(DisplayPortBase& displayPort) // cppcheck-suppress constParameterCallback
{
    (void)displayPort;

    const ReceiverBase::controls_pwm_t controlsPWM = _cockpit.getReceiver().getControlsPWM();
    switch (_RC_CHANNELS_channel) {
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

    if (++_RC_CHANNELS_channel == ReceiverBase::STICK_COUNT) {
        _RC_CHANNELS_channel = 0;
        _activeElement.rendered = true;
    } else {
        // rendering not complete until all 4 channels rendered
        _activeElement.rendered = false;
    }
}

void OSD_Elements::draw_EFFICIENCY(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_TOTAL_FLIGHTS(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_UP_DOWN_REFERENCE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_TX_UPLINK_POWER(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_WATT_HOURS_DRAWN(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_AUX_VALUE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_READY_MODE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_RSNR_VALUE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_SYS(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_GPS_LAP_TIME_CURRENT(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_GPS_LAP_TIME_PREVIOUS(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_GPS_LAP_TIME_BEST3(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_CUSTOM_MSG(DisplayPortBase& displayPort)
{
    (void)displayPort;
}

void OSD_Elements::draw_LIDAR_DISTANCE(DisplayPortBase& displayPort)
{
    (void)displayPort;
}
