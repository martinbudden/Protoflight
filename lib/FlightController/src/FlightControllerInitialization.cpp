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

    enum { DEFAULT_DTERM_FILTER_CUTOFF_FREQUENCY_HZ = 100 };
    const float deltaT = static_cast<float>(taskIntervalMicroSeconds) * 0.000001F;
    _rollRateDTermFilter.setCutoffFrequency(DEFAULT_DTERM_FILTER_CUTOFF_FREQUENCY_HZ, deltaT);
    _pitchRateDTermFilter.setCutoffFrequency(DEFAULT_DTERM_FILTER_CUTOFF_FREQUENCY_HZ, deltaT);

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
