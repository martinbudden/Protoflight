#pragma once

#include "Autopilot.h"
#include "Cockpit.h"
#if defined(USE_OSD)
#include <DisplayPortBase.h>
#endif
#include "FlightController.h"
#include "IMU_Filters.h"
#include <MotorMixerBase.h>
#if defined(USE_OSD)
#include <OSD.h>
#endif
#if defined(USE_VTX)
#include <VTX.h>
#endif

enum units_e { UNITS_METRIC = 0, UNIT_IMPERIAL = 1 };


namespace DEFAULTS {

static constexpr MotorMixerBase::mixer_config_t motorMixerConfig {
    .type = MotorMixerBase::QUAD_X,
    .yaw_motors_reversed = true,
};

static constexpr DynamicIdleController::config_t dynamicIdleControllerConfig = {
    .dyn_idle_min_rpm_100 = 0,
    .dyn_idle_p_gain = 50,
    .dyn_idle_i_gain = 50,
    .dyn_idle_d_gain = 50,
    .dyn_idle_max_increase = 150,
};


static constexpr MotorMixerBase::motor_config_t motorConfig = {
    .device = {
        .motorPWM_Rate = 480, // 16000 for brushed
        .motorProtocol = MotorMixerBase::MOTOR_PROTOCOL_DSHOT300,
        .motorInversion = false,
        .useContinuousUpdate = true,
        .useBurstDshot = 0,
        .useDshotTelemetry = 0,
        .useDshotEDT = 0,
    },
    .motorIdle = 550, // 700 for brushed
    .maxthrottle = 2000,
    .mincommand = 1000,
    .kv = 1960,
    .motorPoleCount = 14,
};

static const FlightController::pidf_uint16_array_t& flightControllerPIDs = FlightController::DefaultPIDs;

static constexpr FlightController::simplified_pid_settings_t flightControllerSimplifiedPID_settings = {
    .multiplier = 100,
    .roll_pitch_ratio = 100,
    .i_gain = 100,
    .d_gain = 100,
    .pi_gain = 100,
    .pitch_pi_gain = 100,
    .d_max_gain = 100,
    .k_gain = 100,
};

static constexpr FlightController::filters_config_t flightControllerFiltersConfig = {
    .dterm_lpf1_hz = 75,
    .dterm_lpf2_hz = 150,
#if defined(USE_DTERM_FILTERS_EXTENDED)
    .dterm_notch_hz = 0,
    .dterm_notch_cutoff = 160,
    .dterm_dynamic_lpf1_min_hz = 75,
    .dterm_dynamic_lpf1_max_hz = 150,
    .dterm_lpf1_type = FlightController::filters_config_t::PT1,
    .dterm_lpf2_type = FlightController::filters_config_t::PT1,
#endif
    .yaw_lpf_hz = 100,
    .output_lpf_hz = 500,
    .rc_smoothing_feedforward_cutoff = 0,
};

static constexpr FlightController::flight_mode_config_t flightControllerFlightModeConfig = {
    .level_race_mode = false,
};

static constexpr FlightController::tpa_config_t flightControllerTPA_Config = {
    .tpa_mode = FlightController::TPA_MODE_D,
    .tpa_rate = 65,
    .tpa_breakpoint = 1350,
    .tpa_low_rate = 20,
    .tpa_low_always = 0,
    .tpa_low_breakpoint = 1050,
};

static constexpr FlightController::anti_gravity_config_t flightControllerAntiGravityConfig = {
    .cutoff_hz = 5,
    .p_gain = 100,
    .i_gain = 80,
};

#if defined(USE_D_MAX)
static constexpr FlightController::d_max_config_t flightControllerDMaxConfig = {
    .d_max = { 40, 46 },
    .d_max_gain = 37,
    .d_max_advance = 20,
};
#endif

#if defined(USE_ITERM_RELAX)
static constexpr FlightController::iterm_relax_config_t flightControllerITermRelaxConfig = {
    .iterm_relax = FlightController::ITERM_RELAX_ON,
    .iterm_relax_setpoint_threshold = 40, // degrees per second
    .iterm_relax_cutoff = 15,
};
#endif

#if defined(USE_YAW_SPIN_RECOVERY)
static constexpr FlightController::yaw_spin_recovery_config_t flightControllerYawSpinRecoveryConfig = {
    .yaw_spin_threshold = 1950,
    .yaw_spin_recovery = FlightController::YAW_SPIN_RECOVERY_OFF,
};
#endif

#if defined(USE_CRASH_RECOVERY)
static constexpr FlightController::crash_recovery_config_t flightControllerCrashRecoveryConfig = {
    .crash_dthreshold = 50,
    .crash_gthreshold = 400,
    .crash_setpoint_threshold = 350,
    .crash_time = 500,
    .crash_delay = 0,
    .crash_limit_yaw = 200,
    .crash_recovery_angle = 10,
    .crash_recovery_rate = 100,
    .crash_recovery = FlightController::CRASH_RECOVERY_OFF,
};
#endif

#if defined(USE_DYNAMIC_NOTCH_FILTER)
static constexpr DynamicNotchFilter::config_t dynamicNotchFilterConfig = {
    .dyn_notch_min_hz = 100,
    .dyn_notch_max_hz = 600,
    .dyn_notch_q = 300,
    .dyn_notch_count = 3,
    .dyn_notch_smoothing = 1,
};
#endif

static constexpr IMU_Filters::config_t imuFiltersConfig = {
    .acc_lpf_hz = 100,
    .gyro_lpf1_hz = 0, // switched off
    .gyro_lpf2_hz = 250, // this is an anti-alias filter and shouldn't be disabled
    .gyro_notch1_hz = 0,
    .gyro_notch1_cutoff = 0,
    .gyro_notch2_hz = 0,
    .gyro_notch2_cutoff = 0,
    //.gyro_dynamic_lpf1_min_hz = 0,
    //.gyro_dynamic_lpf1_max_hz = 0,
    .gyro_lpf1_type = 0,
    .gyro_lpf2_type = IMU_Filters::config_t::PT1,
};

#if defined(USE_RPM_FILTERS)
static constexpr RPM_Filters::config_t rpmFiltersConfig = {
    .rpm_filter_fade_range_hz = 50,
    .rpm_filter_q = 500,
    .rpm_filter_lpf_hz = 150,
    .rpm_filter_weights = { 100, 0, 100 }, // default is not to filter second harmonic
    .rpm_filter_harmonics = 3,
    .rpm_filter_min_hz = 100,
};
#endif

static constexpr Cockpit::rates_t cockpitRates = {
    .rateLimits = { Cockpit::RATE_LIMIT_MAX, Cockpit::RATE_LIMIT_MAX, Cockpit::RATE_LIMIT_MAX},
    .rcRates = { 7, 7, 7 },
    .rcExpos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttleMidpoint = 50,
    .throttleExpo = 0,
    .throttleLimitType = Cockpit::THROTTLE_LIMIT_TYPE_OFF,
    .throttleLimitPercent = 100,
    //.ratesType = Cockpit::RATES_TYPE_ACTUAL
};

static constexpr Cockpit::failsafe_config_t cockpitFailSafeConfig = {
    .throttle_pwm = 1000, // throttle off
    .throttle_low_delay_deciseconds = 100,
    .recovery_delay_deciseconds = 5,
    .delay_deciseconds = 15,
    .landing_time_seconds = 60,
    .procedure = Cockpit::FAILSAFE_PROCEDURE_DROP_IT,
    .switch_mode = Cockpit::FAILSAFE_SWITCH_MODE_STAGE1,
    .stick_threshold_percent = 30,
};

static constexpr Cockpit::rx_config_t cockpitRX_Config = {
    .serial_rx_type = Cockpit::SERIAL_RX_CRSF,
    .serial_rx_inverted = 0,
    .half_duplex = 0,
    .rssi_channel = 0,
    .rssi_scale = 100,
    .rssi_invert = 0,
    .rssi_offset = 0,
    .fpvCamAngleDegrees = 0,        // Camera angle to be scaled into rc commands
    .airModeActivateThreshold = 25, // Throttle setpoint percent where airmode gets activated
    .spektrum_sat_bind = 0,         // number of bind pulses for Spektrum satellite receivers
    .mid_rc = 1500,                 // Some radios have not a neutral point centered on 1500
    .min_check = 1050,
    .max_check = 1900,
    .rx_min_usec = 885,
    .rx_max_usec = 2115,
};

#if defined(USE_ALTITUDE_HOLD)
static constexpr Autopilot::autopilot_config_t autopilotConfig = {
    .altitudePID = { 15, 15, 15, 0, 15 },
    .positionPID = { 30, 30, 30, 0, 30 },
    .landingAltitudeMeters = 4,
    .throttle_hover_pwm = 1275,
    .throttle_min_pwm = 1100,
    .throttle_max_pwm = 1700,
    .position_lpf_hz100 = 80, // cutoff frequency*100 for longitude and latitude position filters
    .maxAngle = 50,
};

static constexpr Autopilot::position_config_t autopilotPositionConfig = {
    .altitude_lpf_hz100 = 300,          // lowpass cutoff Hz*100 for altitude smoothing
    .altitude_dterm_lpf_hz100 = 100,    // lowpass cutoff Hz*100 for altitude derivative smoothing
    .altitude_source = Autopilot::DEFAULT_SOURCE,
    .altitude_prefer_baro = 100,        // percentage trust of barometer data
};

static constexpr Autopilot::altitude_hold_config_t autopilotAltitudeHoldConfig = {
    .climbRate = 50,    // max vertical velocity change at full/zero throttle. 50 means 5 m/s
    .deadband = 20,     // throttle deadband in percent of stick travel
};
#endif

#if defined(USE_OSD)
static constexpr OSD::config_t osdConfig = {
    .profile = {},
    .rcChannels = { -1, -1, -1, -1 },
    .timers = {},
    .enabled_warnings_flags = static_cast<uint32_t>(~(OSD::WARNING_RSSI |OSD::WARNING_LINK_QUALITY | OSD::WARNING_RSSI_DBM |OSD::WARNING_RSNR | OSD::WARNING_OVER_CAP)),
/*
        WARNING_ARMING_DISABLE,
        WARNING_BATTERY_NOT_FULL,
        WARNING_BATTERY_WARNING,
        WARNING_BATTERY_CRITICAL,
        WARNING_VISUAL_BEEPER,
        WARNING_CRASHFLIP,
        WARNING_ESC_FAIL,
        WARNING_CORE_TEMPERATURE,
        WARNING_RC_SMOOTHING,
        WARNING_FAIL_SAFE,
        WARNING_LAUNCH_CONTROL,
        WARNING_GPS_RESCUE_UNAVAILABLE,
        WARNING_GPS_RESCUE_DISABLED,
        WARNING_LOAD,
        WARNING_POSHOLD_FAILED,
*/
    .enabled_stats_flags = static_cast<uint32_t>(~(OSD::STATS_MAX_SPEED | OSD::STATS_MIN_BATTERY | OSD::STATS_MIN_RSSI | OSD::STATS_MAX_CURRENT | OSD::STATS_USED_MAH | OSD::STATS_BLACKBOX | OSD::STATS_BLACKBOX_NUMBER | OSD::STATS_TIMER_2)),
/*
        STATS_RTC_DATE_TIME,
        STATS_TIMER_1,
        STATS_MAX_DISTANCE,
        STATS_END_BATTERY,
        STATS_BATTERY,
        STATS_MAX_ALTITUDE,
        STATS_MAX_G_FORCE,
        STATS_MAX_ESC_TEMPERATURE,
        STATS_MAX_ESC_RPM,
        STATS_MIN_LINK_QUALITY,
        STATS_FLIGHT_DISTANCE,
        STATS_MAX_FFT,
        STATS_TOTAL_FLIGHTS,
        STATS_TOTAL_TIME,
        STATS_TOTAL_DISTANCE,
        STATS_MIN_RSSI_DBM,
        STATS_WATT_HOURS_DRAWN,
        STATS_MIN_RSNR,
        STATS_BEST_3_CONSEC_LAPS,
        STATS_BEST_LAP,
        STATS_FULL_THROTTLE_TIME,
        STATS_FULL_THROTTLE_COUNTER,
        STATS_AVG_THROTTLE,
*/
    .framerate_hz = OSD::FRAMERATE_DEFAULT_HZ,
    .cap_alarm  = 2200,
    .alt_alarm  = 100, // meters or feet depend on configuration
    .link_quality_alarm = 80,
    .rssi_dbm_alarm = -60,
    .rsnr_alarm = 4,
    .distance_alarm = 0,
    .esc_rpm_alarm = OSD::ESC_RPM_ALARM_OFF,
    .esc_current_alarm = OSD::ESC_CURRENT_ALARM_OFF,
    .esc_temperature_alarm = OSD::ESC_TEMPERATURE_ALARM_OFF,
    .core_temperature_alarm = 70, // a temperature above 70C should produce a warning, lockups have been reported above 80C
    .rssi_alarm = 20,

    .units = UNITS_METRIC,

    .aux_scale = 200,
    .aux_channel = 1,
    .aux_symbol = 'A',

    .logo_on_arming = OSD::LOGO_ARMING_OFF,
    .logo_on_arming_duration = 5,  // 0.5 seconds
    .arming_logo_attribute = 0,

    .ahMaxPitch = 20, // 20 degrees
    .ahMaxRoll = 40, // 40 degrees
    .ahInvert = false,

    .osdProfileIndex = 0,
    .overlay_radio_mode = 2,
    .gps_sats_show_pdop = false,

    .camera_frame_width = 24,
    .camera_frame_height = 11,

    .cms_background_type = DisplayPortBase::BACKGROUND_TRANSPARENT,
    .stats_show_cell_value = false,
    .osd_craftname_messages = false,   // Insert LQ/RSSI-dBm and warnings into CraftName
    // Make it obvious on the configurator that the FC doesn't support HD
#if defined(USE_OSD_HD)
    .displayPortDevice = DisplayPortBase::DEVICE_TYPE_MSP,
    .canvas_column_count = OSD::HD_COLS,
    .canvas_row_count = OSD::HD_ROWS,
#else
    .display_port_device_type = DisplayPortBase::DEVICE_TYPE_AUTO,
    .canvas_column_count = OSD::SD_COLS,
    .canvas_row_count = OSD::SD_ROWS,
#endif
    .osd_use_quick_menu = 0,
    .osd_show_spec_prearm = 0,
};

static constexpr OSD_Elements::config_t osdElementsConfig = {};

#endif

#if defined(USE_VTX)
static constexpr VTX::config_t vtxConfig = {
    .frequencyMHz = 5740,
    .pitModeFrequencyMHz = 0,
    .band =4,
    .channel = 1,
    .power = 1,
    .lowPowerDisarm = 1,
    .softserialAlt = 0
};
#endif

} // END namespace
