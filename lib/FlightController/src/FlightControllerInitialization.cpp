#include "FlightController.h"
#include "FlightControllerDefaults.h"

#include <AHRS.h>


/*!
Constructor. Sets member data.
*/
FlightController::FlightController(uint32_t taskIntervalMicroSeconds, const AHRS& ahrs, MotorMixerBase& motorMixer, RadioControllerBase& radioController, Debug& debug) :
    VehicleControllerBase(AIRCRAFT, PID_COUNT, taskIntervalMicroSeconds, ahrs),
    _mixer(motorMixer),
    _radioController(radioController),
    _debug(debug),
    _scaleFactors(gScaleFactors)
{
    for (size_t ii = 0; ii < PID_COUNT; ++ii) {
        _PIDS[ii].setPID(gDefaultPIDs[ii]);
    };
    const filters_config_t filtersConfig = {
        .dterm_lpf1_hz = 100,
        .dterm_lpf2_hz = 0,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff =160,
        .dterm_dynamic_lpf1_min_hz = 0,
        .dterm_dynamic_lpf1_max_hz = 0,
        .yaw_lpf_hz = 0,
        .dterm_lpf1_type = filters_config_t::PT1,
        .dterm_lpf2_type = filters_config_t::PT1
    };
    setFiltersConfig(filtersConfig);

    const float deltaT = static_cast<float>(taskIntervalMicroSeconds) * 0.000001F;
    _rollRateDTermFilter.setCutoffFrequency(filtersConfig.dterm_lpf1_hz, deltaT);
    _pitchRateDTermFilter.setCutoffFrequency(filtersConfig.dterm_lpf1_hz, deltaT);
    _rollAngleDTermFilter.setCutoffFrequency(filtersConfig.dterm_lpf1_hz, deltaT);
    _pitchAngleDTermFilter.setCutoffFrequency(filtersConfig.dterm_lpf1_hz, deltaT);

#if false
    // set the motor output filters to passthrough, by default
    _outputFilters[ROLL_RATE_DPS].setToPassthrough();
    _outputFilters[PITCH_RATE_DPS].setToPassthrough();
    _outputFilters[YAW_RATE_DPS].setToPassthrough();
#else
    enum { DEFAULT_OUTPUT_FILTER_CUTOFF_FREQUENCY_HZ = 500 };
    const float ahrsDeltaT = static_cast<float>(ahrs.getTaskIntervalMicroSeconds()) * 0.000001F;
    _outputFilters[ROLL_RATE_DPS].setCutoffFrequency(DEFAULT_OUTPUT_FILTER_CUTOFF_FREQUENCY_HZ, ahrsDeltaT);
    _outputFilters[PITCH_RATE_DPS].setCutoffFrequency(DEFAULT_OUTPUT_FILTER_CUTOFF_FREQUENCY_HZ, ahrsDeltaT);
    _outputFilters[YAW_RATE_DPS].setCutoffFrequency(DEFAULT_OUTPUT_FILTER_CUTOFF_FREQUENCY_HZ, ahrsDeltaT);
#endif
}
