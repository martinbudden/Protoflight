#include "FlightController.h"
#include "FlightControllerDefaults.h"


/*!
Constructor. Sets member data.
*/
FlightController::FlightController(uint32_t taskIntervalMicroSeconds, const AHRS& ahrs, MotorMixerBase& motorMixer, RadioControllerBase& radioController) :
    VehicleControllerBase(AIRCRAFT, PID_COUNT, taskIntervalMicroSeconds, ahrs),
    _mixer(motorMixer),
    _radioController(radioController),
    _scaleFactors(gScaleFactors)
{
    for (size_t ii = 0; ii < PID_COUNT; ++ii) {
        _PIDS[ii].setPID(gDefaultPIDs[ii]);
    };

    enum { DEFAULT_DTERM_FILTER_CUTOFF_FREQUENCY_HZ = 100 };
    const float deltaT = static_cast<float>(taskIntervalMicroSeconds) * 0.000001F;
    _rollRateDTermFilter.setCutoffFrequency(DEFAULT_DTERM_FILTER_CUTOFF_FREQUENCY_HZ, deltaT);
    _pitchRateDTermFilter.setCutoffFrequency(DEFAULT_DTERM_FILTER_CUTOFF_FREQUENCY_HZ, deltaT);
}
