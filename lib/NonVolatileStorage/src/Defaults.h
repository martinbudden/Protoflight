#pragma once

#include "Autopilot.h"
#include "Cockpit.h"
#if defined(USE_OSD)
#include <DisplayPortBase.h>
#endif
#include "FlightController.h"
#include "IMU_Filters.h"
#include "RC_Modes.h"
#include <motor_mixer_base.h>
#include <receiver_base.h>

#if defined(USE_GPS)
#include <GPS.h>
#endif
#if defined(USE_OSD)
#include <OSD.h>
#endif
#if defined(USE_VTX)
#include <VTX.h>
#endif

namespace DEFAULTS {

static constexpr features_config_t FEATURES_CONFIG {
    .enabledFeatures = Features::FEATURE_RX_SERIAL | Features::FEATURE_ANTI_GRAVITY | Features::FEATURE_AIRMODE
};

static constexpr mixer_config_t MOTOR_MIXER_CONFIG {
    .type = MotorMixerBase::QUAD_X,
    .yaw_motors_reversed = true,
};

static constexpr dynamic_idle_controller_config_t DYNAMIC_IDLE_CONTROLLER_CONFIG = {
    .dyn_idle_min_rpm_100 = 0,
    .dyn_idle_p_gain = 50,
    .dyn_idle_i_gain = 50,
    .dyn_idle_d_gain = 50,
    .dyn_idle_max_increase = 150,
};

static constexpr motor_config_t MOTOR_CONFIG = {
    .device = {
        .motor_pwm_rate = 480, // 16000 for brushed
        .motor_protocol = MotorMixerBase::MOTOR_PROTOCOL_DSHOT300,
        .motor_inversion = false,
        .use_continuous_update = true,
        .use_burst_dshot = 0,
        .use_dshot_telemetry = 0,
        .use_dshot_edt = 0,
    },
    .motor_idle = 550, // 700 for brushed
    .max_throttle = 2000,
    .min_command = 1000,
    .kv = 1960,
    .motor_pole_count = 14,
};

static constexpr FlightController::default_pids_t& FLIGHT_CONTROLLER_PIDS = FlightController::DefaultPIDs;

static constexpr simplified_pid_settings_t FLIGHT_CONTROLLER_SIMPLIFIED_PID_SETTINGS = {
    .multiplier = 100,
    .roll_pitch_ratio = 100,
    .i_gain = 100,
    .d_gain = 100,
    .pi_gain = 100,
    .pitch_pi_gain = 100,
    .d_max_gain = 100,
    .k_gain = 100,
};

static constexpr flight_controller_filters_config_t FLIGHT_CONTROLLER_FILTERS_CONFIG = {
    .dterm_lpf1_hz = 75,
    .dterm_lpf2_hz = 150,
#if defined(USE_DTERM_FILTERS_EXTENDED)
    .dterm_notch_hz = 0,
    .dterm_notch_cutoff = 160,
    .dterm_dynamic_lpf1_min_hz = 75,
    .dterm_dynamic_lpf1_max_hz = 150,
    .dterm_lpf1_type = flight_controller_filters_config_t::PT1,
    .dterm_lpf2_type = flight_controller_filters_config_t::PT1,
#endif
    .yaw_lpf_hz = 100,
    .output_lpf_hz = 500,
    .rc_smoothing_feedforward_cutoff = 0,
};

static constexpr flight_mode_config_t FLIGHT_CONTROLLER_FLIGHT_MODE_CONFIG = {
    .level_race_mode = false,
};

static constexpr tpa_config_t FLIGHT_CONTROLLER_TPA__CONFIG = {
    .tpa_mode = FlightController::TPA_MODE_D,
    .tpa_rate = 65,
    .tpa_breakpoint = 1350,
    .tpa_low_rate = 20,
    .tpa_low_always = 0,
    .tpa_low_breakpoint = 1050,
};

static constexpr anti_gravity_config_t FLIGHT_CONTROLLER_ANTI_GRAVITY_CONFIG = {
    .cutoff_hz = 5,
    .p_gain = 100,
    .i_gain = 80,
};

static constexpr crash_flip_config_t FLIGHT_CONTROLLER_CRASH_FLIP_CONFIG = {
    .motor_percent = 0,
    .rate = 0,
    .auto_rearm = false,
};

#if defined(USE_D_MAX)
static constexpr d_max_config_t FLIGHT_CONTROLLER_D_MAX_CONFIG = {
    .d_max = { 40, 46 },
    .d_max_gain = 37,
    .d_max_advance = 20,
};
#endif

#if defined(USE_ITERM_RELAX)
static constexpr iterm_relax_config_t FLIGHT_CONTROLLER_ITERM_RELAX_CONFIG = {
    .iterm_relax_type = 0,
    .iterm_relax = FlightController::ITERM_RELAX_ON,
    .iterm_relax_setpoint_threshold = 40, // degrees per second
    .iterm_relax_cutoff = 15,
};
#endif

#if defined(USE_YAW_SPIN_RECOVERY)
static constexpr yaw_spin_recovery_config_t FLIGHT_CONTROLLER_YAW_SPIN_RECOVERY_CONFIG = {
    .yaw_spin_threshold = 1950,
    .yaw_spin_recovery = FlightController::YAW_SPIN_RECOVERY_OFF,
};
#endif

#if defined(USE_CRASH_RECOVERY)
static constexpr crash_recovery_config_t FLIGHT_CONTROLLER_CRASH_RECOVERY_CONFIG = {
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
static constexpr dynamic_notch_filter_config_t DYNAMIC_NOTCH_FILTER_CONFIG = {
    .dyn_notch_min_hz = 100,
    .dyn_notch_max_hz = 600,
    .dyn_notch_q = 300,
    .dyn_notch_count = 3,
    .dyn_notch_smoothing = 1,
};
#endif

static constexpr imu_filters_config_t IMU_FILTERS_CONFIG = {
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
    .gyro_lpf2_type = imu_filters_config_t::PT1,
};

#if defined(USE_RPM_FILTERS)
static constexpr rpm_filters_config_t RPM_FILTERS_CONFIG = {
    .rpm_filter_fade_range_hz = 50,
    .rpm_filter_q = 500,
    .rpm_filter_lpf_hz = 150,
    .rpm_filter_weights = { 100, 0, 100 }, // default is not to filter second harmonic
    .rpm_filter_harmonics = 3,
    .rpm_filter_min_hz = 100,
};
#endif

static constexpr rates_t COCKPIT_RATES = {
    .rateLimits = { rates_t::LIMIT_MAX, rates_t::LIMIT_MAX, rates_t::LIMIT_MAX},
    .rcRates = { 7, 7, 7 },
    .rcExpos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttleMidpoint = 50,
    .throttleExpo = 0,
    .throttleLimitType = rates_t::THROTTLE_LIMIT_TYPE_OFF,
    .throttleLimitPercent = 100,
    //.ratesType = rates_t::RATES_TYPE_ACTUAL
};

static constexpr failsafe_config_t COCKPIT_FAILSAFE_CONFIG = {
    .throttle_pwm = 1000, // throttle off
    .throttle_low_delay_deciseconds = 100,
    .recovery_delay_deciseconds = 5,
    .delay_deciseconds = 15,
    .landing_time_seconds = 60,
    .procedure = Cockpit::FAILSAFE_PROCEDURE_DROP_IT,
    .switch_mode = Cockpit::FAILSAFE_SWITCH_MODE_STAGE1,
    .stick_threshold_percent = 30,
};

static constexpr rx_config_t RX_CONFIG = {
    // .rc_map = {},
    .serial_rx_provider = RX::SERIAL_CRSF,
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

/*!
Mode activation conditions.

By default AUX1 is for arming, AUX2 for angle mode, and AUX3 for altitude hold.
*/
static constexpr rc_modes_activation_condition_array_t RC_MODE_ACTIVATION_CONDITIONS = {{
    {
        .range_start = RcModes::RANGE_STEP_MID,
        .range_end = RcModes::RANGE_STEP_MAX,
        .mode_id = MspBox::BOX_ARM,
        .auxiliary_channel_index = ReceiverBase::AUX1 - ReceiverBase::AUX1, // NOLINT(misc-redundant-expression)
        .mode_logic = {},
        .linked_to = {}
    }, {
        .range_start = RcModes::RANGE_STEP_MID,
        .range_end = RcModes::RANGE_STEP_MAX,
        .mode_id = MspBox::BOX_ANGLE,
        .auxiliary_channel_index = ReceiverBase::AUX2 - ReceiverBase::AUX1,
        .mode_logic = {},
        .linked_to = {}
    }, {
        .range_start = RcModes::RANGE_STEP_MID,
        .range_end = RcModes::RANGE_STEP_MAX,
        .mode_id = MspBox::BOX_ALTITUDE_HOLD,
        .auxiliary_channel_index = ReceiverBase::AUX2 - ReceiverBase::AUX1,
        .mode_logic = {},
        .linked_to = {}
    },
    {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}
}};

static constexpr rc_adjustment_ranges_t RC_ADJUSTMENT_RANGES = {};

static constexpr RC_Adjustments::adjustment_configs_t RC_ADJUSTMENT_CONFIGS = {{
    {
        .adjustment = ADJUSTMENT_RC_RATE,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_RC_EXPO,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_THROTTLE_EXPO,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_PITCH_ROLL_RATE,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_YAW_RATE,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_PITCH_ROLL_P,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_PITCH_ROLL_I,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_PITCH_ROLL_D,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_YAW_P,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_YAW_I,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_YAW_D,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_RATE_PROFILE,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 3 }
    }, {
        .adjustment = ADJUSTMENT_PITCH_RATE,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_ROLL_RATE,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_PITCH_P,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_PITCH_I,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_PITCH_D,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_ROLL_P,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_ROLL_I,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_ROLL_D,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_YAW_RC_RATE,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_PITCH_ROLL_K,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_FEEDFORWARD_TRANSITION,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_HORIZON_STRENGTH,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 255 }
    }, {
        .adjustment = ADJUSTMENT_PID_AUDIO,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 7 } // ARRAYLEN(pidAudioPositionToModeMap)
    }, {
        .adjustment = ADJUSTMENT_PITCH_K,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_ROLL_K,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_YAW_K,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustment = ADJUSTMENT_OSD_PROFILE,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 3 }
    }, {
        .adjustment = ADJUSTMENT_LED_PROFILE,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 3 }
    }, {
        .adjustment = ADJUSTMENT_LED_DIMMER,
        .mode = RC_Adjustments::ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 100 }
    },
    {
        .adjustment = ADJUSTMENT_NONE,
        .mode = {},
        .data = {}
    },
    {}, {}, {}
}};

#if defined(USE_ALTITUDE_HOLD)
static constexpr autopilot_config_t AUTOPILOT_CONFIG = {
    .altitudePID = { 15, 15, 15, 0, 15 },
    .positionPID = { 30, 30, 30, 0, 30 },
    .landingAltitudeMeters = 4,
    .throttle_hover_pwm = 1275,
    .throttle_min_pwm = 1100,
    .throttle_max_pwm = 1700,
    .position_lpf_hz100 = 80, // cutoff frequency*100 for longitude and latitude position filters
    .maxAngle = 50,
};

static constexpr position_hold_config_t AUTOPILOT_POSITION_HOLD_CONFIG = {
    .altitude_lpf_hz100 = 300,          // lowpass cutoff Hz*100 for altitude smoothing
    .altitude_dterm_lpf_hz100 = 100,    // lowpass cutoff Hz*100 for altitude derivative smoothing
    .altitude_source = Autopilot::DEFAULT_SOURCE,
    .altitude_prefer_baro = 100,        // percentage trust of barometer data
};

static constexpr altitude_hold_config_t AUTOPILOT_ALTITUDE_HOLD_CONFIG = {
    .climbRate = 50,    // max vertical velocity change at full/zero throttle. 50 means 5 m/s
    .deadband = 20,     // throttle deadband in percent of stick travel
};
#endif

#if defined(USE_OSD)
static constexpr osd_config_t OSD_CONFIG = {
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

    .units = OSD::UNITS_METRIC,

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

static constexpr osd_elements_config_t OSD_ELEMENTS_CONFIG = {};

#endif

#if defined(USE_VTX)
static constexpr vtx_config_t VTX_CONFIG = {
    .frequencyMHz = 5740,
    .pitModeFrequencyMHz = 0,
    .band =4,
    .channel = 1,
    .power = 1,
    .lowPowerDisarm = 1,
    .softserialAlt = 0
};
#endif

#if defined(USE_GPS)
static constexpr gps_config_t GPS_CONFIG = {
#if defined(USE_VIRTUAL_GPS)
    .provider = GPS_VIRTUAL,
#else
    .provider = GPS::GPS_UBLOX,
#endif
    .sbasMode = GPS::SBAS_NONE,
    .auto_CONFIG = GPS::AUTO_CONFIG_ON,
    .autoBaud = GPS::AUTO_BAUD_OFF,
    .gps_ublox_acquire_model = GPS::MODEL_STATIONARY,
    .gps_ublox_flight_model = GPS::MODEL_AIRBORNE_4G,
    .gps_update_rate_hz = 10,
    .gps_ublox_use_galileo = false,
    .gps_set_home_point_once = false,
    .gps_use_3d_speed = false,
    .sbas_integrity = false,
    .gps_ublox_utc_standard = GPS::UTC_STANDARD_AUTO,
};
#endif

} // END namespace
