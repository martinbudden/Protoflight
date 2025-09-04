#include "FlightController.h"

#include <AHRS.h>
#include <PIDF.h>


constexpr FlightController::pidf_array_t flightControllerDefaultPIDs = {
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
constexpr FlightController::pidf_array_t flightControllerScaleFactors = {
    {
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // roll rate
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // pitch rate
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // yaw rate
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // roll angle
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // pitch angle
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }, // roll sin angle
        { 0.01F,    0.01F,   0.001F, 0.1F, 0.1F }  // pitch sin angle
    }
};

/*!
Constructor. Sets member data.
*/
FlightController::FlightController(uint32_t taskDenominator, const AHRS& ahrs, MotorMixerBase& motorMixer, RadioControllerBase& radioController, Debug& debug) :
    VehicleControllerBase(AIRCRAFT, PID_COUNT, ahrs.getTaskIntervalMicroSeconds() / taskDenominator, ahrs),
    _mixer(motorMixer),
    _radioController(radioController),
    _debug(debug),
    _taskDenominator(taskDenominator),
    _scaleFactors(flightControllerScaleFactors)
{
    for (size_t ii = 0; ii < PID_COUNT; ++ii) {
        _PIDS[ii].setPID(flightControllerDefaultPIDs[ii]);
    };
}
