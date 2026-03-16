#include "autopilot.h"
#include "cockpit.h"
#include "display_port_base.h"
#include "flight_controller.h"
#include "osd.h"
#include "osd_symbols.h"
#include "rc_modes.h"
#include "vtx.h"

#include <ahrs_message_queue.h>
#include <cstdio>
#include <debug.h>
#include <gps.h>
#include <gps_message_queue.h>
#include <receiver_base.h>


//
// Drawing functions
//

std::array<OSD_Elements::elementDrawFnPtr, OSD_ELEMENT_COUNT> OSD_Elements::DrawFunctions {};
std::array<OSD_Elements::elementDrawFnPtr, OSD_ELEMENT_COUNT> OSD_Elements::DrawBackgroundFunctions {};


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
#if defined(USE_RTC_TIME)
    DrawFunctions[OSD_RTC_DATETIME]     = &OSD_Elements::draw_RTC_DATETIME;
#endif
#if defined(USE_OSD_ADJUSTMENTS)
    DrawFunctions[OSD_ADJUSTMENT_RANGE] = &OSD_Elements::draw_ADJUSTMENT_RANGE;
#endif
    DrawFunctions[OSD_CORE_TEMPERATURE] = &OSD_Elements::draw_CORE_TEMPERATURE;
    DrawFunctions[OSD_ANTI_GRAVITY]     = &OSD_Elements::draw_ANTI_GRAVITY;
    DrawFunctions[OSD_G_FORCE]          = &OSD_Elements::draw_G_FORCE;
#if defined(USE_DSHOT)
    DrawFunctions[OSD_MOTOR_DIAGNOSTICS] = &OSD_Elements::draw_MOTOR_DIAGNOSTICS;
#endif
    DrawFunctions[OSD_LOG_STATUS]       = &OSD_Elements::draw_LOG_STATUS;
    DrawFunctions[OSD_FLIP_ARROW]       = &OSD_Elements::draw_FLIP_ARROW;
    DrawFunctions[OSD_LINK_QUALITY]     = &OSD_Elements::draw_LINK_QUALITY;
#if defined(USE_GPS)
    DrawFunctions[OSD_FLIGHT_DISTANCE]  = &OSD_Elements::draw_FLIGHT_DISTANCE;
#endif
    DrawFunctions[OSD_STICK_OVERLAY_LEFT] = &OSD_Elements::draw_STICK_OVERLAY;
    DrawFunctions[OSD_STICK_OVERLAY_RIGHT] = &OSD_Elements::draw_STICK_OVERLAY;
#if defined(USE_OSD_PROFILES)
    DrawFunctions[OSD_PILOT_NAME]       = &OSD_Elements::draw_PILOT_NAME;
#endif
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
#if defined(USE_OSD_HD) && defined(USE_MSP_DISPLAYPORT)
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
#if defined(USE_OSD_PROFILES)
    DrawFunctions[OSD_CUSTOM_MSG0]      = &OSD_Elements::draw_CUSTOM_MSG;
    DrawFunctions[OSD_CUSTOM_MSG1]      = &OSD_Elements::draw_CUSTOM_MSG;
    DrawFunctions[OSD_CUSTOM_MSG2]      = &OSD_Elements::draw_CUSTOM_MSG;
    DrawFunctions[OSD_CUSTOM_MSG3]      = &OSD_Elements::draw_CUSTOM_MSG;
#endif
#if defined(USE_RANGEFINDER)
    DrawFunctions[OSD_LIDAR_DISTANCE]   = &OSD_Elements::draw_LIDAR_DISTANCE;
#endif

    DrawBackgroundFunctions[OSD_HORIZON_SIDEBARS]   = &OSD_Elements::drawBackground_HORIZON_SIDEBARS;
    DrawBackgroundFunctions[OSD_CRAFT_NAME]         = &OSD_Elements::drawBackground_CRAFT_NAME;
    DrawBackgroundFunctions[OSD_STICK_OVERLAY_LEFT] = &OSD_Elements::drawBackground_STICK_OVERLAY;
    DrawBackgroundFunctions[OSD_STICK_OVERLAY_RIGHT]= &OSD_Elements::drawBackground_STICK_OVERLAY;
#if defined(USE_OSD_PROFILES)
    DrawBackgroundFunctions[OSD_PILOT_NAME]         = &OSD_Elements::drawBackground_PILOT_NAME;
#endif
    DrawBackgroundFunctions[OSD_CAMERA_FRAME]       = &OSD_Elements::drawBackground_CAMERA_FRAME;
};

void OSD_Elements::format_distance_string(char* buf, float distance, char leadingSymbol) // NOLINT(readability-non-const-parameter)
{
    float converted_distance = distance;
    float unit_transition = 1000.0F;
    char unit_symbol = SYM_M;
    char unit_symbol_extended = SYM_KM;

    static constexpr float METERS_TO_FEET = 3.28084F;
    if (_osd.get_config().units == OSD::UNITS_IMPERIAL) {
        converted_distance = static_cast<float>(distance) * METERS_TO_FEET;
        unit_transition = 5280;
        unit_symbol = SYM_FT;
        unit_symbol_extended = SYM_MILES;
    }

    unsigned decimal_places = 0;
    float display_distance = converted_distance;
    char display_symbol = unit_symbol;
    if (converted_distance >= unit_transition) {
        display_distance = converted_distance / unit_transition;
        display_symbol = unit_symbol_extended;
        if (display_distance >= 10) { // >= 10 miles or km - 1 decimal place
            decimal_places = 1;
        } else {                     // < 10 miles or km - 2 decimal places
            decimal_places = 2;
        }
    }
    printFloat(buf, leadingSymbol, display_distance, decimal_places, false, display_symbol);
}

void OSD_Elements::formatPID(const osd_context_t& ctx, char* buf, const char* label, uint8_t axis) // NOLINT(readability-non-const-parameter)
{
    const pid_constants_uint16_t pid = ctx.flight_controller.get_pid_msp(axis);
    sprintf(buf, "%s %3d %3d %3d %3d", label,
        pid.kp,
        pid.ki,
        pid.kd,
        pid.ks
        //pid.kk
    );
}

void OSD_Elements::drawBackground_HORIZON_SIDEBARS(const osd_context_t& ctx)
{
    // Draw AH sides
    const int8_t width = AH_SIDEBAR_WIDTH_POS;
    const int8_t height = AH_SIDEBAR_HEIGHT_POS;

    if (_HORIZON_SIDEBARS_RenderLevel) {
        // AH level indicators
        ctx.display_port.write_char(_active_element.pos_x - width + 1, _active_element.pos_y, SYM_AH_LEFT);
        ctx.display_port.write_char(_active_element.pos_x + width - 1, _active_element.pos_y, SYM_AH_RIGHT);
        _HORIZON_SIDEBARS_RenderLevel = false;
    } else {
        ctx.display_port.write_char(_active_element.pos_x - width, _active_element.pos_y + static_cast<uint8_t>(_HORIZON_SIDEBARS__pos_y), SYM_AH_DECORATION);
        ctx.display_port.write_char(_active_element.pos_x + width, _active_element.pos_y + static_cast<uint8_t>(_HORIZON_SIDEBARS__pos_y), SYM_AH_DECORATION);
        if (_HORIZON_SIDEBARS__pos_y == height) {
            // Rendering is complete, so prepare to start again
            _HORIZON_SIDEBARS__pos_y = -height;
            // On next pass render the level markers
            _HORIZON_SIDEBARS_RenderLevel = true;
        } else {
            ++_HORIZON_SIDEBARS__pos_y;
        }
        // Rendering not yet complete
        _active_element.rendered = false;
    }
    _active_element.draw_element = false;  // element already drawn
}

void OSD_Elements::drawBackground_CRAFT_NAME(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::drawBackground_STICK_OVERLAY(const osd_context_t& ctx)
{
    (void)ctx;

    if (_STICK_OVERLAY_RenderPhase == VERTICAL) {
        sprintf(&_active_element.buf[0], "%c", SYM_STICK_OVERLAY_VERTICAL);
        _active_element.offset_x = ((STICK_OVERLAY_WIDTH - 1) / 2);
        _active_element.offset_y = _STICK_OVERLAY_Y;

        ++_STICK_OVERLAY_Y;

        if (_STICK_OVERLAY_Y == (STICK_OVERLAY_HEIGHT - 1) / 2) {
            // Skip over horizontal
            ++_STICK_OVERLAY_Y;
        }

        if (_STICK_OVERLAY_Y == STICK_OVERLAY_HEIGHT) {
            _STICK_OVERLAY_Y = 0;
            _STICK_OVERLAY_RenderPhase = HORIZONTAL;
        }

        _active_element.rendered = false;
    } else {
        for (uint8_t i = 0; i < STICK_OVERLAY_WIDTH; i++) {
            _active_element.buf[i] = SYM_STICK_OVERLAY_HORIZONTAL;
        }
        _active_element.buf[((STICK_OVERLAY_WIDTH - 1) / 2)] = SYM_STICK_OVERLAY_CENTER;
        _active_element.buf[STICK_OVERLAY_WIDTH] = 0;  // string terminator

        _active_element.offset_y = ((STICK_OVERLAY_HEIGHT - 1) / 2);

        _STICK_OVERLAY_RenderPhase = VERTICAL;
    }
}

void OSD_Elements::drawBackground_PILOT_NAME(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::drawBackground_CAMERA_FRAME(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_RSSI_VALUE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_MAIN_BATTERY_VOLTAGE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_CROSSHAIRS(const osd_context_t& ctx)
{
    (void)ctx;
    _active_element.buf[0] = SYM_AH_CENTER_LINE;
    _active_element.buf[1] = SYM_AH_CENTER;
    _active_element.buf[2] = SYM_AH_CENTER_LINE_RIGHT;
    _active_element.buf[3] = 0;
}

void OSD_Elements::draw_ARTIFICIAL_HORIZON(const osd_context_t& ctx)
{
    (void)ctx;

    enum { AH_SYMBOL_COUNT = 9 };
    enum { AH_SYMBOL_SIDE_COUNT = 4 };
    static int x = -AH_SYMBOL_SIDE_COUNT;
    // Get pitch and roll limits in tenths of degrees
    const float max_pitch = static_cast<float>(_osd.get_config().ah_max_pitch) * 10.0F;
    const float max_roll = static_cast<float>(_osd.get_config().ah_max_roll) * 10.0F;
    const float ahSign = _osd.get_config().ah_invert ? -1.0F : 1.0F;
    const float rollAngle = std::clamp(_roll_angle_degrees * ahSign, -max_roll, max_roll);
    float pitchAngle = std::clamp(_pitch_angle_degrees * ahSign, -max_pitch, max_pitch);
    // Convert pitchAngle to y compensation value
    // (max_pitch / 25) divisor matches previous settings of fixed divisor of 8 and fixed max AHI pitch angle of 20.0 degrees
    if (max_pitch > 0.0F) {
        pitchAngle = ((pitchAngle * 25.0F) / max_pitch);
    }
    pitchAngle -= 4 * AH_SYMBOL_COUNT + 5;

    const int y = ((-static_cast<int>(rollAngle) * x) / 64) - static_cast<int>(pitchAngle);
    if (y >= 0 && y <= 81) {
        _active_element.offset_x = static_cast<uint8_t>(x);
        _active_element.offset_y = static_cast<uint8_t>(y / AH_SYMBOL_COUNT);
#if defined(M5_UNIFIED)
        _active_element.buf[0] = '-';
        _active_element.buf[1] = 0;
#else
        sprintf(&_active_element.buf[0], "%c", (SYM_AH_BAR9_0 + (y % AH_SYMBOL_COUNT)));
#endif
    } else {
        _active_element.draw_element = false;  // element does not need to be rendered
    }

    if (x == AH_SYMBOL_SIDE_COUNT) {
        // Rendering is complete, so prepare to start again
        x = -AH_SYMBOL_SIDE_COUNT;
    } else {
        // Rendering not yet complete
        _active_element.rendered = false;
        ++x;
    }
}

void OSD_Elements::draw_ITEM_TIMER(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_FLYMODE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_THROTTLE_POS(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_VTX_CHANNEL(const osd_context_t& ctx)
{
    (void)ctx;

    if (ctx.vtx == nullptr) {
        return;
    }
    const vtx_config_t vtxConfig = ctx.vtx->get_config();
    uint8_t band = vtxConfig.band;
    uint8_t channel = vtxConfig.channel;
    if (band == 0) {
        // Direct frequency set is used
        ctx.vtx->lookup_band_channel(band, channel, vtxConfig.frequency_mhz);
    }
    const char vtxBandLetter = VTX::lookup_band_letter(band);
    const char* vtx_channel_name = VTX::lookup_channel_name(channel);
    uint32_t vtxStatus = 0;
    ctx.vtx->get_status(vtxStatus);

    uint8_t vtxPower = vtxConfig.power;
    if (vtxConfig.lowPowerDisarm) {
        ctx.vtx->get_power_index(vtxPower);
    }
    const char* vtxPowerLabel = ctx.vtx->lookup_power_name(vtxPower);
    char vtxStatusIndicator = '\0';
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_VTX_CONTROL_DISABLE)) {
        vtxStatusIndicator = 'D';
    } else if (vtxStatus & VTX::STATUS_PIT_MODE) {
        vtxStatusIndicator = 'P';
    }

    switch (_active_element.type) {
    case OSD_ELEMENT_TYPE_2:
        sprintf(&_active_element.buf[0], "%s", vtxPowerLabel);
        break;

    default:
        if (vtxStatus & VTX::STATUS_LOCKED) {
            sprintf(&_active_element.buf[0], "-:-:-:L");
        } else if (vtxStatusIndicator) {
            sprintf(&_active_element.buf[0], "%c:%s:%s:%c", vtxBandLetter, vtx_channel_name, vtxPowerLabel, vtxStatusIndicator);
        } else {
            sprintf(&_active_element.buf[0], "%c:%s:%s", vtxBandLetter, vtx_channel_name, vtxPowerLabel);
        }
        break;
    }
}

void OSD_Elements::draw_CURRENT_DRAW(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_MAH_DRAWN(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_GPS_SPEED(const osd_context_t& ctx)
{
    (void)ctx;
#if defined(USE_GPS)
    static constexpr float CMPS_TO_KMPH = 36.0F / 1000.0F;
    static constexpr float CMPS_TO_MPH = 10000.0F / 5080.0F / 88.0F;

    uint8_t speedSymbol = SYM_KPH;
    float speedConversionFactor = CMPS_TO_KMPH;
    if (_osd.get_config().units != OSD::UNITS_METRIC) {
        speedSymbol = SYM_MPH;
        speedConversionFactor = CMPS_TO_MPH;
    }
    gps_message_data_t gps_message_data {};
    ctx.gps->getGPS_MessageQueue().PEEK_GPS_DATA(gps_message_data);
    if (gps_message_data.fix & gps_message_data_t::FIX) {
        //speed = gpsConfig()->gps_use_3d_speed ? gpsSol.speed3d : gpsSol.ground_speed)
        const int speed = static_cast<int>(static_cast<float>(gps_message_data.ground_speed_cmps) * speedConversionFactor);
        sprintf(&_active_element.buf[0], "%c%3d%c", SYM_SPEED, speed, speedSymbol);
    } else {
        sprintf(&_active_element.buf[0], "%c%c%c", SYM_SPEED, SYM_HYPHEN, speedSymbol);
    }
#endif
}

void OSD_Elements::draw_GPS_SATS(const osd_context_t& ctx)
{
    (void)ctx;
#if defined(USE_GPS)
    if (ctx.gps == nullptr) {
        return;
    }
    gps_message_data_t gps_message_data {};
    ctx.gps->getGPS_MessageQueue().PEEK_GPS_DATA(gps_message_data);
    [[maybe_unused]] const gps_rescue_config_t& gpsRescueConfig = ctx.cockpit.get_autopilot().getGPS_RescueConfig();
    enum { GPS_SATELLITE_COUNT_CRITICAL =  4 };
    if ((gps_message_data.fix == 0) || (gps_message_data.satellite_count < GPS_SATELLITE_COUNT_CRITICAL) ) {
        _active_element.attr = DisplayPortBase::SEVERITY_CRITICAL;
    }
#if defined(USE_GPS_RESCUE)
    else if ((gps_message_data.satellite_count < gpsRescueConfig.minSats) && ctx.cockpit.gps_rescue_is_configured()) {
        _active_element.attr = DisplayPortBase::SEVERITY_WARNING;
    }
#endif
    else {
        _active_element.attr = DisplayPortBase::SEVERITY_NORMAL;
    }

    if (gps_message_data.is_healthy) {
        const size_t pos = static_cast<size_t>(printf(&_active_element.buf[0], "%c%c%2u", SYM_SAT_L, SYM_SAT_R, gps_message_data.satellite_count));
        if (_osd.get_config().gps_sats_show_pdop) { // add on the GPS module PDOP estimate
            _active_element.buf[pos] = ' ';
            printFloat(&_active_element.buf[pos + 1], SYM_NONE, gps_message_data.dilution_of_precision_positional / 100.0F, 1, true, SYM_NONE);
        }
    } else {
        sprintf(&_active_element.buf[0], "%c%cNC", SYM_SAT_L, SYM_SAT_R);
    }
#endif
}

void OSD_Elements::draw_ALTITUDE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_ROLL_PIDS(const osd_context_t& ctx)
{
    (void)ctx;
    formatPID(ctx, &_active_element.buf[0], "PIDR", FlightController::ROLL_RATE_DPS);
}

void OSD_Elements::draw_PITCH_PIDS(const osd_context_t& ctx)
{
    (void)ctx;
    formatPID(ctx, &_active_element.buf[0], "PIDP", FlightController::PITCH_RATE_DPS);
}

void OSD_Elements::draw_YAW_PIDS(const osd_context_t& ctx)
{
    (void)ctx;
    formatPID(ctx, &_active_element.buf[0], "PIDY", FlightController::YAW_RATE_DPS);
}

void OSD_Elements::draw_POWER(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_PID_RATE_PROFILE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_WARNINGS(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_AVG_CELL_VOLTAGE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_GPS_LAT_LONG(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_DEBUG(const osd_context_t& ctx)
{
    (void)ctx;
    sprintf(&_active_element.buf[0], "DBG %5d %5d %5d %5d", ctx.debug.get(0), ctx.debug.get(1),ctx.debug.get(2),ctx.debug.get(3));
}

void OSD_Elements::draw_DEBUG2(const osd_context_t& ctx)
{
    (void)ctx;
    sprintf(&_active_element.buf[0], "DBG %5d %5d %5d %5d", ctx.debug.get(4), ctx.debug.get(5),ctx.debug.get(6),ctx.debug.get(7));
}

void OSD_Elements::draw_PITCH_ANGLE(const osd_context_t& ctx)
{
    (void)ctx;
#if defined(M5_UNIFIED)
    sprintf(&_active_element.buf[0], "p:%4d", static_cast<int>(_pitch_angle_degrees));
#else
    sprintf(&_active_element.buf[0], "PIT%4d", static_cast<int>(_pitch_angle_degrees));
#endif
}

void OSD_Elements::draw_ROLL_ANGLE(const osd_context_t& ctx)
{
    (void)ctx;
#if defined(M5_UNIFIED)
    sprintf(&_active_element.buf[0], "r:%4d", static_cast<int>(_roll_angle_degrees));
#else
    sprintf(&_active_element.buf[0], "ROL%4d", static_cast<int>(_roll_angle_degrees));
#endif
}

void OSD_Elements::draw_MAIN_BATTERY_USAGE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_DISARMED(const osd_context_t& ctx)
{
    (void)ctx;
    sprintf(&_active_element.buf[0], ctx.cockpit.is_armed()? "ARMED" : "DISARMED");
}

void OSD_Elements::draw_HOME_DIRECTION(const osd_context_t& ctx)
{
    (void)ctx;(void)ctx;
}

void OSD_Elements::draw_HOME_DISTANCE(const osd_context_t& ctx)
{
    (void)ctx;(void)ctx;
}

void OSD_Elements::draw_NUMERICAL_HEADING(const osd_context_t& ctx)
{
    (void)ctx;(void)ctx;
#if defined(M5_UNIFIED)
    sprintf(&_active_element.buf[0], "y:%4d", static_cast<int>(_yaw_angle_degrees));
#else
    sprintf(&_active_element.buf[0], "PIT%4d", static_cast<int>(_yaw_angle_degrees));
#endif
}

void OSD_Elements::draw_NUMERICAL_VARIO(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_COMPASS_BAR(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_ESC_TEMPERATURE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_ESC_RPM(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_REMAINING_TIME_ESTIMATE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_RTC_DATETIME(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_ADJUSTMENT_RANGE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_CORE_TEMPERATURE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_ANTI_GRAVITY(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_G_FORCE(const osd_context_t& ctx)
{
    (void)ctx;
    //OSD::printFloat(&_active_element.buf[0], SYM_NONE, osdGForce, 1, true, 'G');
}

void OSD_Elements::draw_MOTOR_DIAGNOSTICS(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_LOG_STATUS(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_FLIP_ARROW(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_LINK_QUALITY(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_FLIGHT_DISTANCE(const osd_context_t& ctx) // NOLINT(readability-make-member-function-const)
{
    (void)ctx;
#if defined(USE_GPS)
    if (ctx.gps == nullptr) {
        return;
    }
    gps_message_data_t gps_message_data {};
    ctx.gps->getGPS_MessageQueue().PEEK_GPS_DATA(gps_message_data);
    if ((gps_message_data.fix & gps_message_data_t::FIX) && (gps_message_data.fix & gps_message_data_t::FIX_HOME)) {
        format_distance_string(&_active_element.buf[0], gps_message_data.distance_flown_meters, SYM_TOTAL_DISTANCE);
    } else {
        // We use this symbol when we don't have a FIX
        sprintf(&_active_element.buf[0], "%c%c", SYM_TOTAL_DISTANCE, SYM_HYPHEN);
    }
#endif
}

void OSD_Elements::draw_STICK_OVERLAY(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_PILOT_NAME(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_ESC_RPM_FREQUENCY(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_RATE_PROFILE_NAME(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_PID_PROFILE_NAME(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_PROFILE_NAME(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_RSSI_DBM_VALUE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_RC_CHANNELS(const osd_context_t& ctx) // cppcheck-suppress constParameterCallback
{
    (void)ctx;

    const receiver_controls_pwm_t controls_pwm = ctx.receiver.get_controls_pwm();
    switch (_RC_CHANNELS_channel) {
    case 0:
        sprintf(&_active_element.buf[0], "T:%5d", controls_pwm.throttle);
        _active_element.offset_x = 0;
        _active_element.offset_y = 0;
        break;
    case 1:
        sprintf(&_active_element.buf[0], "R:%5d", controls_pwm.roll);
        _active_element.offset_x = ctx.display_port.get_column_count()/2;
        _active_element.offset_y = 0;
        break;
    case 2:
        sprintf(&_active_element.buf[0], "P:%5d", controls_pwm.pitch);
        _active_element.offset_x = ctx.display_port.get_column_count()/2;
        _active_element.offset_y = 1;
        break;
    default:
        sprintf(&_active_element.buf[0], "Y:%5d", controls_pwm.yaw);
        _active_element.offset_x = 0;
        _active_element.offset_y = 1;
        break;
    }

    if (++_RC_CHANNELS_channel == ReceiverBase::STICK_COUNT) {
        _RC_CHANNELS_channel = 0;
        _active_element.rendered = true;
    } else {
        // rendering not complete until all 4 channels rendered
        _active_element.rendered = false;
    }
}

void OSD_Elements::draw_EFFICIENCY(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_TOTAL_FLIGHTS(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_UP_DOWN_REFERENCE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_TX_UPLINK_POWER(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_WATT_HOURS_DRAWN(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_AUX_VALUE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_READY_MODE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_RSNR_VALUE(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_SYS(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_GPS_LAP_TIME_CURRENT(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_GPS_LAP_TIME_PREVIOUS(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_GPS_LAP_TIME_BEST3(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_CUSTOM_MSG(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD_Elements::draw_LIDAR_DISTANCE(const osd_context_t& ctx)
{
    (void)ctx;
}

int OSD_Elements::printFloat(char* buffer, char leadingSymbol, float value, unsigned decimal_places, bool round, char trailingSymbol)
{
    int pos = 0;
    int multiplier = 1;
    for (size_t ii = 0; ii < decimal_places; ++ii) {
        multiplier *= 10;
    }

    value *= static_cast<float>(multiplier);
    const int scaledValueAbs = std::abs(round ? static_cast<int>(lrintf(value)) : static_cast<int>(value));
    const int integerPart = scaledValueAbs / multiplier;
    const int fractionalPart = scaledValueAbs % multiplier;

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    if (leadingSymbol != SYM_NONE) {
        buffer[pos++] = leadingSymbol;
    }
    if (value < 0 && (integerPart || fractionalPart)) {
        buffer[pos++] = '-';
    }

    pos += sprintf(buffer + pos, "%01d", integerPart);
    if (decimal_places) {
        std::array<char, 16> mask {};
        sprintf(&mask[0], ".%%0%uu", decimal_places); // builds up the format string to be like ".%03u" for decimal_places == 3 as an example
        pos += sprintf(buffer + pos, &mask[0], fractionalPart);
    }

    if (trailingSymbol != SYM_NONE) {
        buffer[pos++] = trailingSymbol;
    }
    buffer[pos] = '\0';
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    return pos;
}
