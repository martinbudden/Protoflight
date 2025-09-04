#include "FlightController.h"

#include <AHRS.h>
#include <PIDF.h>


const std::array<PIDF::PIDF_t, FlightController::PID_COUNT> FlightController::_scaleFactors = {
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
    _taskDenominator(taskDenominator)
{
}
