#pragma once

#include <DynamicIdleController.h>
#include <FlightController.h>
#include <IMU_Filters.h>
#include <RPM_Filters.h>
#include <RadioController.h>


namespace DEFAULTS {


/*!
Default PIDs.
Same values as used by Betaflight.
*/
static constexpr FlightController::pidf_uint16_array_t flightControllerDefaultPIDs = {{
    { 45, 80, 30, 120, 0 }, // roll rate
    { 47, 84, 34, 125, 0 }, // pitch rate
    { 45, 80,  0, 120, 0 }, // yaw rate
    { 50, 75, 75,  50, 0 }, // roll angle
    { 50, 75, 75,  50, 0 }, // pitch angle
    { 50, 75, 75,  50, 0 }, // roll sin angle
    { 50, 75, 75,  50, 0 }, // pitch sin angle
}};

static const DynamicIdleController::config_t dynamicIdleControllerConfig = {
    .dyn_idle_min_rpm_100 = 0,
    .dyn_idle_p_gain = 50,
    .dyn_idle_i_gain = 50,
    .dyn_idle_d_gain = 50,
    .dyn_idle_max_increase = 150,
};

static const FlightController::filters_config_t flightControllerFiltersConfig = {
    .dterm_lpf1_hz = 75,
    .dterm_lpf2_hz = 150,
    .dterm_notch_hz = 0,
    .dterm_notch_cutoff = 160,
    .dterm_dynamic_lpf1_min_hz = 75,
    .dterm_dynamic_lpf1_max_hz = 150,
    .yaw_lpf_hz = 100,
    .output_lpf_hz = 500,
    .dterm_lpf1_type = FlightController::filters_config_t::PT1,
    .dterm_lpf2_type = FlightController::filters_config_t::PT1,
    .rc_smoothing_feedforward_cutoff = 0,
};

static const FlightController::tpa_config_t flightControllerTPA_Config = {
    .tpa_mode = FlightController::TPA_MODE_D,
    .tpa_rate = 65,
    .tpa_breakpoint = 1350,
    .tpa_low_rate = 20,
    .tpa_low_always = 0,
    .tpa_low_breakpoint = 1050,
};

static const FlightController::anti_gravity_config_t flightControllerAntiGravityConfig = {
    .cutoff_hz = 5,
    .p_gain = 100,
    .i_gain = 80
};

#if defined(USE_D_MAX)
static const FlightController::d_max_config_t flightControllerDMaxConfig = {
    .d_max = { 40, 46 },
    .d_max_gain = 37,
    .d_max_advance = 20
};
#endif

#if defined(USE_ITERM_RELAX)
static const FlightController::iterm_relax_config_t flightControllerITermRelaxConfig = {
    .iterm_relax = FlightController::ITERM_RELAX_ON,
    .iterm_relax_setpoint_threshold = 40, // degrees per second
    .iterm_relax_cutoff = 15,
};
#endif

#if defined(USE_YAW_SPIN_RECOVERY)
static const FlightController::yaw_spin_recovery_config_t flightControllerYawSpinRecoveryConfig = {
    .yaw_spin_threshold = 1950,
    .yaw_spin_recovery = FlightController::YAW_SPIN_RECOVERY_OFF,
};
#endif

#if defined(USE_CRASH_RECOVERY)
static const FlightController::crash_recovery_config_t flightControllerCrashRecoveryConfig = {
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

static const IMU_Filters::config_t imuFiltersConfig = {
    .gyro_notch1_hz = 0,
    .gyro_notch1_cutoff = 0,
    .gyro_notch2_hz = 0,
    .gyro_notch2_cutoff = 0,
    .gyro_lpf1_hz = 0, // switched off
    .gyro_lpf2_hz = 250, // this is an anti-alias filter and shouldn't be disabled
    .gyro_dynamic_lpf1_min_hz = 0,
    .gyro_dynamic_lpf1_max_hz = 0,
    .gyro_lpf1_type = 0,
    .gyro_lpf2_type = IMU_Filters::config_t::PT1,
};

static const RPM_Filters::config_t rpmFiltersConfig = {
    .rpm_filter_harmonics = 3,
    .rpm_filter_weights = { 100, 0, 100 }, // default is not to filter second harmonic
    .rpm_filter_min_hz = 100,
    .rpm_filter_fade_range_hz = 50,
    .rpm_filter_q = 500,
    .rpm_filter_lpf_hz = 150,
};

static const RadioController::failsafe_t radioControllerFailsafe = {
    .delay = 15,
    .landing_time = 60,
    .switch_mode = 0,
    .procedure = 0,
    .throttle = 1000,
    .throttle_low_delay = 100
};

static const RadioController::rates_t radioControllerRates = {
    .rateLimits = { RadioController::RATE_LIMIT_MAX, RadioController::RATE_LIMIT_MAX, RadioController::RATE_LIMIT_MAX},
    .rcRates = { 7, 7, 7 },
    .rcExpos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttleMidpoint = 50,
    .throttleExpo = 0,
    .throttleLimitType = RadioController::THROTTLE_LIMIT_TYPE_OFF,
    .throttleLimitPercent = 100,
    //.ratesType = RadioController::RATES_TYPE_ACTUAL
};


} // END namespace
