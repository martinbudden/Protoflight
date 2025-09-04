#include "FlightController.h"

#include <AHRS.h>
#include <Blackbox.h> // just needed for //_blackbox->finish() and endlog()

#if !defined(UNIT_TEST_BUILD)
//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif
#endif
#include <RadioController.h>
#include <ReceiverBase.h>
#include <TimeMicroSeconds.h>

#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <esp32-hal.h>
#endif

#if defined(FRAMEWORK_USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
inline void YIELD_TASK() { taskYIELD(); }
#else
inline void YIELD_TASK() {}
#endif


static const std::array<std::string, FlightController::PID_COUNT> PID_NAMES = {
    "ROLL_RATE",
    "PITCH_RATE",
    "YAW_RATE",
    "ROLL_ANGLE",
    "PITCH_ANGLE"
};

const std::string& FlightController::getPID_Name(pid_index_e pidIndex) const
{
    return PID_NAMES[pidIndex];
}

/*!
Set the P, I, D, and F values for the PID with index pidIndex.
Integration is switched off, so that there is no integral windup before takeoff.
*/
void FlightController::setPID_Constants(pid_index_e pidIndex, const PIDF::PIDF_t& pid)
{
    _PIDS[pidIndex].setPID(pid);
    _PIDS[pidIndex].switchIntegrationOff();
}

uint32_t FlightController::getOutputPowerTimeMicroSeconds() const
{
    //return _mixer.getOutputPowerTimeMicroSeconds();
    return 0;
}

VehicleControllerBase::PIDF_uint16_t FlightController::getPID_MSP(size_t index) const
{
    assert(index < PID_COUNT);

    const auto pidIndex = static_cast<pid_index_e>(index);
    const PIDF_uint16_t ret = {
        .kp = static_cast<uint16_t>(_PIDS[pidIndex].getP() / _scaleFactors[pidIndex].kp),
        .ki = static_cast<uint16_t>(_PIDS[pidIndex].getI() / _scaleFactors[pidIndex].ki),
        .kd = static_cast<uint16_t>(_PIDS[pidIndex].getD() / _scaleFactors[pidIndex].kd),
        .kf = static_cast<uint16_t>(_PIDS[pidIndex].getF() / _scaleFactors[pidIndex].kf),
        .ks = static_cast<uint16_t>(_PIDS[pidIndex].getS() / _scaleFactors[pidIndex].ks),
    };
    return ret;
}

bool FlightController::isArmingFlagSet(arming_flag_e armingFlag) const
{
    (void)armingFlag;
    return true; // !!TODO arming flag
}

bool FlightController::isFlightModeFlagSet(flight_mode_flag_e flightModeFlag) const
{
    (void)flightModeFlag;
    return true; // !!TODO flight mode flag
}

bool FlightController::isRcModeActive(uint8_t rcMode) const
{
    (void)rcMode;
    return true; // !!TODO rcMode
}

float FlightController::getBatteryVoltage() const
{
    return 11.6F;
}

float FlightController::getAmperage() const
{
    return 0.67F;
}

void FlightController::motorsSwitchOff()
{
    _mixer.motorsSwitchOff();
    _takeOffCountStart = 0;
    _groundMode = true;
    switchPID_integrationOff();
    if (_blackbox) {
        _blackbox->endLog();
        //_blackbox->finish();
    }
}

void FlightController::motorsSwitchOn()
{
    // don't allow motors to be switched on if the sensor fusion has not initialized
    if (!_ahrs.sensorFusionFilterIsInitializing()) {
        _mixer.motorsSwitchOn();
        // reset the PID integral values when we switch the motors on
        switchPID_integrationOn();
        //if (_blackbox) {
        //    _blackbox->start();
        //}
    }
}

void FlightController::motorsToggleOnOff()
{
    if (motorsIsOn()) {
        motorsSwitchOff();
    } else {
        motorsSwitchOn();
    }
}

/*!
Sets the control mode.
*/
void FlightController::setControlMode(control_mode_e controlMode)
{
    if (controlMode == _controlMode) {
        return;
    }
    _controlMode = controlMode;
    // reset the PID integral values when we change control mode
    for (auto& pid : _PIDS) {
        pid.resetIntegral();
    }
}

void FlightController::setFiltersConfig(const filters_config_t& filtersConfig)
{
    _filtersConfig = filtersConfig;
    //!!TODO: check dT value for filters config
    const float dT = static_cast<float>(_taskIntervalMicroSeconds) / 1000000.0F;
    //const float deltaT = (static_cast<float>(_ahrs.getTaskIntervalMicroSeconds()) * 0.000001F) / static_cast<float>(_taskDenominator);

    if (filtersConfig.dterm_lpf1_hz == 0) {
        _rollRateDTermFilter.setToPassthrough();
        _pitchRateDTermFilter.setToPassthrough();
    } else {
        // if the user has selected a filter, then provide a PowerTransfer1 filter.
        // If no filter selected, then set the filter to passthrough
        switch (filtersConfig.dterm_lpf1_type) {
        case filters_config_t::PT2:
            [[fallthrough]];
        case filters_config_t::PT3:
            [[fallthrough]];
        case filters_config_t::BIQUAD:
            [[fallthrough]];
        case filters_config_t::PT1:
            _rollRateDTermFilter.setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, dT);
            _rollAngleDTermFilter.setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, dT);
            _pitchRateDTermFilter.setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, dT);
            _pitchAngleDTermFilter.setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, dT);
            break;
        default:
            _rollRateDTermFilter.setToPassthrough();
            _rollAngleDTermFilter.setToPassthrough();
            _pitchRateDTermFilter.setToPassthrough();
            _pitchAngleDTermFilter.setToPassthrough();
            break;
        }
    }
    if (filtersConfig.output_lpf_hz == 0) {
        _outputFilters[ROLL_RATE_DPS].setToPassthrough();
        _outputFilters[PITCH_RATE_DPS].setToPassthrough();
        _outputFilters[YAW_RATE_DPS].setToPassthrough();
    } else {
        const float ahrsDeltaT = static_cast<float>(_ahrs.getTaskIntervalMicroSeconds()) * 0.000001F;
        _outputFilters[ROLL_RATE_DPS].setCutoffFrequency(filtersConfig.output_lpf_hz, ahrsDeltaT);
        _outputFilters[PITCH_RATE_DPS].setCutoffFrequency(filtersConfig.output_lpf_hz, ahrsDeltaT);
        _outputFilters[YAW_RATE_DPS].setCutoffFrequency(filtersConfig.output_lpf_hz, ahrsDeltaT);
    }
}

/*!
Return he FC telemetry data.

The telemetry data is shared between this task (the FC task) and the MAIN_LOOP_TASK, however it is deliberately not protected by a mutex.
This is because:
1. Only this task writes to the telemetry data. The main task only reads it (for display and to send to the backchannel).
2. The only inconsistency that can occur is that the main task might use a partially updated telemetry object, however this is not a problem because
   all the member data are continuous, and so a partially updated object is still meaningful to display.
3. The overhead of a mutex is thus avoided.
*/
flight_controller_quadcopter_telemetry_t FlightController::getTelemetryData() const
{
    flight_controller_quadcopter_telemetry_t telemetry;

    for (size_t ii = 0; ii < _mixer.getMotorCount(); ++ ii) {
        telemetry.motors[ii].power = _mixer.getMotorOutput(ii);
        telemetry.motors[ii].rpm = _mixer.getMotorRPM(ii);
    }
    if (motorsIsOn()) {
        const PIDF::error_t rollRateError = _PIDS[ROLL_RATE_DPS].getError();
        telemetry.rollRateError = { rollRateError.P, rollRateError.I, rollRateError.D, rollRateError.F, rollRateError.S };

        const PIDF::error_t pitchRateError = _PIDS[PITCH_RATE_DPS].getError();
        telemetry.pitchRateError = { pitchRateError.P, pitchRateError.I, pitchRateError.D, pitchRateError.F, pitchRateError.S };

        const PIDF::error_t yawRateError = _PIDS[YAW_RATE_DPS].getError();
        telemetry.yawRateError = { yawRateError.P, yawRateError.I, yawRateError.D, yawRateError.F, yawRateError.S };

        const PIDF::error_t rollAngleError = _PIDS[ROLL_ANGLE_DEGREES].getError();
        telemetry.rollAngleError = { rollAngleError.P, rollAngleError.I, rollAngleError.D, rollAngleError.F, rollAngleError.S };

        const PIDF::error_t pitchAngleError = _PIDS[PITCH_ANGLE_DEGREES].getError();
        telemetry.pitchAngleError = { pitchAngleError.P, pitchAngleError.I, pitchAngleError.D, pitchAngleError.F, pitchAngleError.S };
    } else {
        telemetry.rollRateError =  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.pitchRateError = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.yawRateError =   { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.rollAngleError = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.pitchAngleError ={ 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
    }
    return telemetry;
}

/*!
Use the new joystick values from the receiver to update the PID setpoints
using the NED (North-East-Down) coordinate convention.

NOTE: this function is called form `updateControls()` in the ReceiverTask loop() function,
as a result of receiving new values from the receiver.
How often it is called depends on the type of transmitter and receiver the user has,
but is typically at intervals of between 40 milliseconds and 5 milliseconds (ie 25Hz to 200Hz).
In particular it runs much less frequently than `updateOutputsUsingPIDs()` which typically runs at 1000Hz to 8000Hz.
*/
void FlightController::updateSetpoints(const controls_t& controls)
{
    detectCrashOrSpin();

    setControlMode(controls.controlMode);

    //!!TODO: put a critical section around this
    _outputThrottle = controls.throttleStick;
    // adjust the Throttle PID Attenuation (TPA)
    // _TPA is 1.0F (ie no attenuation) if throttleStick <= _TPA_Breakpoint;
    _TPA = 1.0F - _TPA_multiplier * std::fminf(0.0F, controls.throttleStick - _TPA_breakpoint);

    //!!TODO: filter the roll and stick angles
    // Pushing the ROLL stick to the right gives a positive value of rollStick and we want this to be left side up.
    // For NED left side up is positive roll, so sign of setpoint is same sign as rollStick.
    // So sign of _rollStick is left unchanged.
    _PIDS[ROLL_RATE_DPS].setSetpoint(controls.rollStickDPS);
    _rollStickSinAngle = sinf(controls.rollStickDegrees * degreesToRadians);
    _PIDS[ROLL_ANGLE_DEGREES].setSetpoint(controls.rollStickDegrees);

    // Pushing the  PITCH stick forward gives a positive value of _pitchStick and we want this to be nose up.
    // For NED nose up is positive pitch, so sign of setpoint is opposite sign as _pitchStick.
    // So sign of _pitchStick is negated.
    _PIDS[PITCH_RATE_DPS].setSetpoint(-controls.pitchStickDPS);
    _pitchStickSinAngle = sinf(-controls.pitchStickDegrees * degreesToRadians);
    _PIDS[PITCH_ANGLE_DEGREES].setSetpoint(-controls.pitchStickDegrees);

    // Pushing the YAW stick to the right gives a positive value of _yawStick and we want this to be nose right.
    // For NED nose left is positive yaw, so sign of setpoint is same as sign of _yawStick.
    // So sign of _yawStick is left unchanged.
    _PIDS[YAW_RATE_DPS].setSetpoint(controls.yawStickDPS);

    // When in ground mode, the PID I-terms are set to zero to avoid integral windup on the ground
    if (_groundMode) {
        // exit ground mode if the throttle has been above _takeOffThrottleThreshold for _takeOffTickThreshold ticks
        if (_outputThrottle < _takeOffThrottleThreshold) {
            _takeOffCountStart = 0;
        } else {
            const uint32_t tickCount = controls.tickCount;
            if (_takeOffCountStart == 0) {
                _takeOffCountStart = tickCount;
            }
            if (tickCount - _takeOffCountStart > _takeOffTickThreshold) {
                _groundMode = false;
                // we've exited ground mode, so we can turn on PID integration
                switchPID_integrationOn();
            }
        }
    }
    // Angle Mode is used if the controlMode is set to angle mode, or failsafe is on.
    // Angle Mode is prevented when in Ground Mode, so the aircraft doesn't try and self-level while it is still on the ground.
    // This value is cached here, to avoid evaluating a reasonably complex condition in updateOutputsUsingPIDs()
    _useAngleMode = (_controlMode == CONTROL_MODE_ANGLE || (_radioController.getFailsafePhase() != RadioController::FAILSAFE_IDLE)) && !_groundMode;
}

/*!
Detect crash or yaw spin. Runs in context of Receiver Task.
*/
void FlightController::detectCrashOrSpin()
{
    if (_yawSpinThresholdDPS !=0.0F && fabsf(_PIDS[YAW_RATE_DPS].getPreviousMeasurement()) > _yawSpinThresholdDPS) {
        // yaw spin detected
        _yawSpinRecovery = true;
        switchPID_integrationOff();
    }
}

void FlightController::recoverFromYawSpin(const xyz_t& gyroENU_RPS, float deltaT)
{
    if (fabsf(gyroENU_RPS.z) > _yawSpinRecoveredRPS) {
        _outputThrottle = 0.5F; // half throttle gives maximum yaw authority, since outputs will have maximum range before being clipped
        // use the YAW_RATE_DPS PID to bring the spin down to zero
        _PIDS[YAW_RATE_DPS].setSetpoint(0.0F);
        const float yawRateDPS = yawRateNED_DPS(gyroENU_RPS);
        _outputs[YAW_RATE_DPS] = _PIDS[YAW_RATE_DPS].update(yawRateDPS, deltaT);

        if (fabsf(gyroENU_RPS.z) > _yawSpinPartiallyRecoveredRPS) {
            // we are at a high spin rate, so don't yet attempt to recover the roll and pitch spin
            _outputs[ROLL_RATE_DPS] = 0.0F;
            _outputs[PITCH_RATE_DPS] = 0.0F;
        } else {
            // we have partially recovered from the spin, so try and also correct any roll and pitch spin
            _PIDS[ROLL_RATE_DPS].setSetpoint(0.0F);
            const float rollRateDPS = rollRateNED_DPS(gyroENU_RPS);
            _outputs[ROLL_RATE_DPS] = _PIDS[ROLL_RATE_DPS].update(rollRateDPS, deltaT);

            _PIDS[PITCH_RATE_DPS].setSetpoint(0.0F);
            const float pitchRateDPS = pitchRateNED_DPS(gyroENU_RPS);
            _outputs[PITCH_RATE_DPS] = _PIDS[PITCH_RATE_DPS].update(pitchRateDPS, deltaT);
        }
    } else {
        // come out of yaw spin recovery
        _yawSpinRecovery = false;
        // switch PID integration back on
        switchPID_integrationOn();
    }
}


void FlightController::updateOutputsUsingPIDs(float deltaT)
{
    // get orientation, gyro, and acceleration from AHRS in ENU (East-North-Up) format
    const Quaternion orientationENU = _ahrs.getOrientationUsingLock();

    const AHRS::data_t dataENU = _ahrs.getAhrsDataUsingLock();

    updateOutputsUsingPIDs(dataENU.gyroRPS, dataENU.acc, orientationENU, deltaT);
}

/*!
In angle mode, the roll and pitch angles are used to set the setpoints for the rollRate and pitchRate PIDs.
In NFE(Not Fast Enough) Racer mode (aka level race mode) is equivalent to angle mode on roll and acro mode on pitch.
*/
void FlightController::updateRateSetpointsForAngleMode(const Quaternion& orientationENU, float deltaT)
{

    // convert orientationENU from the ENU coordinate frame to the NED coordinate frame
    //static const Quaternion qENUtoNED(0.0F, sqrtf(0.5F), sqrtf(0.5F), 0.0F);
    //const Quaternion orientationNED = qENUtoNED * orientationENU;
    //_rollAngleDegreesRaw = orientationNED.calculateRollDegrees();
    //_pitchAngleDegreesRaw = orientationNED.calculatePitchDegrees();

    const float yawRateSetpointDPS = _PIDS[YAW_RATE_DPS].getSetpoint();

    if (_angleModeUseQuaternionSpace) {
        // Runs the angle PIDs in "quaternion space" rather than "angle space",
        // avoiding the computationally expensive Quaternion::calculateRoll and Quaternion::calculatePitch
        if (_angleModeCalculationState == STATE_CALCULATE_ROLL) {
            if (!_useAngleModeOnRollAcroModeOnPitch) {
                // don't advance calculation to pitch axis when in level race mode
                _angleModeCalculationState = STATE_CALCULATE_PITCH;
            }
            _rollSinAngle = -orientationENU.sinRoll(); // sin(x-180) = -sin(x)
            const float rollSinAngleDelta = _rollAngleDTermFilter.filter(_rollSinAngle - _PIDS[ROLL_SIN_ANGLE].getPreviousMeasurement());
            _outputs[ROLL_SIN_ANGLE] = _PIDS[ROLL_SIN_ANGLE].update(_rollSinAngle, rollSinAngleDelta, deltaT);
            _rollRateSetpointDPS = _outputs[ROLL_SIN_ANGLE];
            // a component of YAW changes roll, so update accordingly !!TODO:check sign
            _rollRateSetpointDPS -= yawRateSetpointDPS * _rollSinAngle;
        } else {
            _angleModeCalculationState = STATE_CALCULATE_ROLL;
            _pitchSinAngle = -orientationENU.sinPitch(); // this is cheaper to calculate than sinRoll
            const float pitchSinAngleDelta = _rollAngleDTermFilter.filter(_pitchSinAngle - _PIDS[PITCH_SIN_ANGLE].getPreviousMeasurement());
            _outputs[PITCH_SIN_ANGLE] = _PIDS[PITCH_SIN_ANGLE].update(_pitchSinAngle, pitchSinAngleDelta, deltaT);
            _pitchRateSetpointDPS = _outputs[PITCH_SIN_ANGLE];
            // a component of YAW changes roll, so update accordingly !!TODO:check sign
            _pitchRateSetpointDPS += yawRateSetpointDPS * _pitchSinAngle;
        }
    } else {
        //!!TODO: this all needs checking, especially for scale
        //!!TODO: PID constants need to be scaled so these outputs are in the range [-1, 1]
        // calculate roll rate and pitch rate setpoints in the NED coordinate frame
        // this is a computationally expensive calculation, so alternate between roll and pitch each time this function is called
        if (_angleModeCalculationState == STATE_CALCULATE_ROLL) {
            if (!_useAngleModeOnRollAcroModeOnPitch) {
                // don't advance calculation to pitch axis when in level race mode
                _angleModeCalculationState = STATE_CALCULATE_PITCH;
            }
            _rollSinAngle = -orientationENU.sinRoll(); // sin(x-180) = -sin(x)
            _rollAngleDegreesRaw = orientationENU.calculateRollDegrees() - 180.0F;
            const float rollAngleDelta = _rollAngleDTermFilter.filter(_rollAngleDegreesRaw - _PIDS[ROLL_ANGLE_DEGREES].getPreviousMeasurement());
            _outputs[ROLL_ANGLE_DEGREES] = _PIDS[ROLL_ANGLE_DEGREES].update(_rollAngleDegreesRaw, rollAngleDelta, deltaT) * _maxRollRateDPS;
            _rollRateSetpointDPS = _outputs[ROLL_ANGLE_DEGREES];
            _rollRateSetpointDPS -= yawRateSetpointDPS * _rollSinAngle;
        } else {
            _angleModeCalculationState = STATE_CALCULATE_ROLL;
            _pitchSinAngle = -orientationENU.sinPitch(); // this is cheaper to calculate than sinRoll
            _pitchAngleDegreesRaw = -orientationENU.calculatePitchDegrees();
            const float pitchAngleDelta = _pitchAngleDTermFilter.filter(_pitchAngleDegreesRaw - _PIDS[PITCH_ANGLE_DEGREES].getPreviousMeasurement());
            _outputs[PITCH_ANGLE_DEGREES] = _PIDS[PITCH_ANGLE_DEGREES].update(_pitchAngleDegreesRaw, pitchAngleDelta, deltaT) * _maxPitchRateDPS;
            _pitchRateSetpointDPS = _outputs[PITCH_ANGLE_DEGREES];
            _pitchRateSetpointDPS += yawRateSetpointDPS * _pitchSinAngle;
        }
    }

    // use the outputs from the "ANGLE" PIDS as the setpoints for the "RATE" PIDs.
    //!!TODO: need to mix in YAW to roll and pitch changes to coordinate turn
    _PIDS[ROLL_RATE_DPS].setSetpoint(_rollRateSetpointDPS);
    _PIDS[PITCH_RATE_DPS].setSetpoint(_pitchRateSetpointDPS);

    // the cosRoll and cosPitch functions are reasonably cheap, they both involve taking a square root
    // both are positive in ANGLE mode, since absolute values of both roll and pitch angles are less than 90 degrees
#if false
    const float rollCosAngle = orientationENU.cosRoll();
    const float pitchCosAngle = orientationENU.cosPitch();
    const float yawRateSetpointAttenuation = fmaxf(rollCosAngle, pitchCosAngle);
#else
    const float rollSinAngle2 = _rollSinAngle*_rollSinAngle;
    const float pitchSinAngle2 = _pitchSinAngle*_pitchSinAngle;
    const float minSinAngle2 = fminf(rollSinAngle2, pitchSinAngle2);
    const float yawRateSetpointAttenuation = sqrtf(1.0F - minSinAngle2); // this is equal to fmaxf(rollCosAngle, pitchCosAngle)
#endif
    // attenuate yaw rate setpoint
    _PIDS[YAW_RATE_DPS].setSetpoint(_PIDS[YAW_RATE_DPS].getSetpoint()*yawRateSetpointAttenuation);
}

/*!
The FlightController uses the NED (North-East-Down) coordinate convention.
gyroRPS, acc, and orientation come from the AHRS and use the ENU (East-North-Up) coordinate convention.

NOTE: this function may be configured to be called either from the FlightController loop() function (ie in the VehicleController task),
or from the AHRS loop() function (ie in the AHRS task).
In the latter case it is typically called at frequency of between 1000Hz and 8000Hz, so it has to be FAST.
*/
void FlightController::updateOutputsUsingPIDs(const xyz_t& gyroENU_RPS, const xyz_t& accENU, const Quaternion& orientationENU, float deltaT)
{
    (void)accENU; // not using acc, since we use the orientation quaternion instead

    if (_yawSpinRecovery) {
        recoverFromYawSpin(gyroENU_RPS, deltaT);
        const VehicleControllerMessageQueue::queue_item_t queueItem {
            .throttle = _outputThrottle,
            .roll = _outputs[ROLL_RATE_DPS],
            .pitch = _outputs[PITCH_RATE_DPS],
            .yaw = _outputs[YAW_RATE_DPS]
        };
        SIGNAL(queueItem);
        return;
    }

    if (_useAngleMode) {
        updateRateSetpointsForAngleMode(orientationENU, deltaT);
    }

    // Use the PIDs to calculate the outputs for each axis.
    // Note that the delta-values (ie the DTerms) are filtered:
    // this is because they are especially noisy, being the derivative of a noisy value.

    const float rollRateDPS = rollRateNED_DPS(gyroENU_RPS);
    const float rollRateDeltaDPS = _rollRateDTermFilter.filter(rollRateDPS - _PIDS[ROLL_RATE_DPS].getPreviousMeasurement());
    _outputs[ROLL_RATE_DPS] = _PIDS[ROLL_RATE_DPS].updateDelta(rollRateDPS, _TPA*rollRateDeltaDPS, deltaT);
    // filter the output
    _outputs[ROLL_RATE_DPS] = _outputFilters[ROLL_RATE_DPS].filter(_outputs[ROLL_RATE_DPS]);

    const float pitchRateDPS = pitchRateNED_DPS(gyroENU_RPS);
    const float pitchRateDeltaDPS = _pitchRateDTermFilter.filter(pitchRateDPS - _PIDS[PITCH_RATE_DPS].getPreviousMeasurement());
    _outputs[PITCH_RATE_DPS] = _PIDS[PITCH_RATE_DPS].updateDelta(pitchRateDPS, _TPA*pitchRateDeltaDPS, deltaT);
    // filter the output
    _outputs[PITCH_RATE_DPS] = _outputFilters[PITCH_RATE_DPS].filter(_outputs[PITCH_RATE_DPS]);

    // DTerm is zero for yawRate, so use updatePI with no DTerm filtering _TPA
    const float yawRateDPS = yawRateNED_DPS(gyroENU_RPS);
    _outputs[YAW_RATE_DPS] = _PIDS[YAW_RATE_DPS].updatePI(yawRateDPS, deltaT);
    // filter the output
    _outputs[YAW_RATE_DPS] = _outputFilters[YAW_RATE_DPS].filter(_outputs[YAW_RATE_DPS]);

    // The VehicleControllerTask is waiting on the message queue, so signal it tha there is output data available.
    // This will result in outputToMixer being called
    const VehicleControllerMessageQueue::queue_item_t queueItem {
        .throttle = _outputThrottle,
        .roll = _outputs[ROLL_RATE_DPS],
        .pitch = _outputs[PITCH_RATE_DPS],
        .yaw = _outputs[YAW_RATE_DPS]
    };
    SIGNAL(queueItem);
}

/*!
Called from within the VehicleControllerTask when signalled that output data is available.
*/
void FlightController::outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem)
{
    ++_taskSignalledCount;
    if (_taskSignalledCount < _taskDenominator) {
        return;
    }
    _taskSignalledCount = 0;

    if (_radioController.getFailsafePhase() == RadioController::FAILSAFE_RX_LOSS_DETECTED) {
        const MotorMixerBase::commands_t commands {
            .throttle  = 0.25F,
            .roll   = 0.0F,
            .pitch  = 0.0F,
            .yaw    = 0.0F
        };
        _mixer.outputToMotors(commands, deltaT, tickCount);
        return;
    }

    const MotorMixerBase::commands_t commands {
        .throttle  = queueItem.throttle,
        // scale roll, pitch, and yaw to range [0.0F, 1.0F]
        .roll   = queueItem.roll / _rollRateAtMaxPowerDPS,
        .pitch  = queueItem.pitch / _pitchRateAtMaxPowerDPS,
        .yaw    = queueItem.yaw / _yawRateAtMaxPowerDPS
    };

    _mixer.outputToMotors(commands, deltaT, tickCount);
}
