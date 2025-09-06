#pragma once

#include <DynamicIdleController.h>
#include <FlightController.h>
#include <IMU_Filters.h>
#include <RPM_Filters.h>
#include <RadioController.h>


namespace DEFAULTS {


static constexpr FlightController::pidf_array_t flightControllerDefaultPIDs = {
    {
        { 0.65F,    0.0F,   0.010F, 0.00F, 0.00F }, // roll rate
        { 0.95F,    0.0F,   0.025F, 0.00F, 0.00F }, // pitch rate
        { 0.50F,    0.0F,   0.010F, 0.00F, 0.00F }, // yaw rate
        { 5.00F,    0.0F,   0.040F, 0.00F, 0.00F }, // roll angle
        { 5.00F,    0.0F,   0.040F, 0.00F, 0.00F }, // pitch angle
        { 1.00F,    0.0F,   0.010F, 0.00F, 0.00F }, // roll sin angle
        { 1.00F,    0.0F,   0.010F, 0.00F, 0.00F }  // pitch sin angle
    }
};

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
    .gyro_hardware_lpf = 0,
    .rpm_filter_harmonics = RPM_Filters::USE_FUNDAMENTAL_ONLY,
    .rpm_filter_min_hz = 100
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
    .ratesType = RadioController::RATES_TYPE_ACTUAL
};


} // END namespace
