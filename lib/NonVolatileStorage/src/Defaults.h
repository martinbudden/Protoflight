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
    .dterm_lpf1_hz = 100,
    .dterm_lpf2_hz = 0,
    .dterm_notch_hz = 0,
    .dterm_notch_cutoff = 160,
    .dterm_dynamic_lpf1_min_hz = 0,
    .dterm_dynamic_lpf1_max_hz = 0,
    .yaw_lpf_hz = 0,
    .dterm_lpf1_type = FlightController::filters_config_t::PT1,
    .dterm_lpf2_type = FlightController::filters_config_t::PT1,
    .output_lpf_hz = 500
};

static const FlightController::anti_gravity_config_t flightControllerAntiGravityConfig = {
    .cutoff_hz = 5,
    .p_gain = 100,
    .i_gain = 80
};

static const FlightController::d_max_config_t flightControllerDMaxConfig = {
    .d_max = { 40, 46, 0 },
    .d_max_gain = 37,
    .d_max_advance = 20
};

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
