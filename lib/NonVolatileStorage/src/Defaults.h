#pragma once

#include <DynamicIdleController.h>
#include <IMU_Filters.h>
#include <RPM_Filters.h>
#include <RadioController.h>


namespace DEFAULTS {

static const IMU_Filters::config_t imuFiltersConfig = {
    .gyro_notch1_hz = 0,
    .gyro_notch1_cutoff = 0,
    .gyro_notch2_hz = 0,
    .gyro_notch2_cutoff = 0,
    .gyro_lpf1_hz = 0, // switched off, alternative is 250
    .gyro_lpf2_hz = 250, // this is an anti-alias filter
    .gyro_dynamic_lpf1_min_hz = 0,
    .gyro_dynamic_lpf1_max_hz = 0,
    .gyro_lpf1_type = 0,
    .gyro_lpf2_type = IMU_Filters::config_t::PT1,
    .gyro_hardware_lpf = 0,
    .rpm_filter_harmonics = RPM_Filters::USE_FUNDAMENTAL_ONLY,
    .rpm_filter_min_hz = 100
};

static const DynamicIdleController::config_t dynamicIdleControllerConfig = {
    .dyn_idle_min_rpm_100 = 0,
    .dyn_idle_p_gain = 50,
    .dyn_idle_i_gain = 50,
    .dyn_idle_d_gain = 50,
    .dyn_idle_max_increase = 150,
};

static const RadioController::rates_t radioControllerRates {
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