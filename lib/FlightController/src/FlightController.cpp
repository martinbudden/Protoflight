#include "Debug.h"
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
#include <TimeMicroseconds.h>

#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <esp32-hal.h>
#endif


/*!
Constructor. Sets member data.
*/
FlightController::FlightController(uint32_t taskDenominator, const AHRS& ahrs, MotorMixerBase& motorMixer, RadioControllerBase& radioController, Debug& debug) :
    VehicleControllerBase(AIRCRAFT, PID_COUNT, ahrs.getTaskIntervalMicroseconds() / taskDenominator, ahrs),
    _mixer(motorMixer),
    _radioController(radioController),
    _debug(debug),
    _taskDenominator(taskDenominator),
    _fcC(_fcM),
    _rxC(_rxM)
{
    _sh.antiGravityThrottleFilter.setToPassthrough();
}

static const std::array<std::string, FlightController::PID_COUNT> PID_NAMES = {
    "ROLL_RATE",
    "PITCH_RATE",
    "YAW_RATE",
    "ROLL_ANGLE",
    "PITCH_ANGLE",
    "ROLL_SIN_ANGLE",
    "PITCH_SIN_ANGLE"
};

const std::string& FlightController::getPID_Name(pid_index_e pidIndex) const
{
    return PID_NAMES[pidIndex];
}


//!!TODO: reconcile FlightController::getPID_MSP, FlightController::getPID_Constants and set variants
VehicleControllerBase::PIDF_uint16_t FlightController::getPID_MSP(size_t index) const
{
    assert(index < PID_COUNT);

    const auto pidIndex = static_cast<pid_index_e>(index);
    const PIDF_uint16_t ret = {
        .kp = static_cast<uint16_t>(_sh.PIDS[pidIndex].getP() / _scaleFactors.kp),
        .ki = static_cast<uint16_t>(_sh.PIDS[pidIndex].getI() / _scaleFactors.ki),
        .kd = static_cast<uint16_t>(_sh.PIDS[pidIndex].getD() / _scaleFactors.kd),
        .kf = static_cast<uint16_t>(_sh.PIDS[pidIndex].getF() / _scaleFactors.kf),
        .ks = static_cast<uint16_t>(_sh.PIDS[pidIndex].getS() / _scaleFactors.ks),
    };
    return ret;
}

VehicleControllerBase::PIDF_uint16_t FlightController::getPID_Constants(pid_index_e pidIndex) const
{
    const PIDF::PIDF_t pid = _sh.PIDS[pidIndex].getPID();
    const VehicleControllerBase::PIDF_uint16_t pid16 = {
        static_cast<uint16_t>(std::lroundf(pid.kp / _scaleFactors.kp)),
        static_cast<uint16_t>(std::lroundf(pid.ki / _scaleFactors.ki)),
        static_cast<uint16_t>(std::lroundf(pid.kd / _scaleFactors.kd)),
        static_cast<uint16_t>(std::lroundf(pid.kf / _scaleFactors.kf)),
        static_cast<uint16_t>(std::lroundf(pid.ks / _scaleFactors.ks))
    };

    return pid16;
}
/*!
Set the P, I, D, and F values for the PID with index pidIndex.
Integration is switched off, so that there is no integral windup before takeoff.
*/
void FlightController::setPID_Constants(pid_index_e pidIndex, const PIDF_uint16_t& pid16)
{

    const PIDF::PIDF_t pid = {
        static_cast<float>(pid16.kp) * _scaleFactors.kp,
        static_cast<float>(pid16.ki) * _scaleFactors.ki,
        static_cast<float>(pid16.kd) * _scaleFactors.kd,
        static_cast<float>(pid16.kf) * _scaleFactors.kf,
        static_cast<float>(pid16.ks) * _scaleFactors.ks
    };

    _sh.PIDS[pidIndex].setPID(pid);
    _sh.PIDS[pidIndex].switchIntegrationOff();
    // keep copies of P and I terms so they can be adjusted by anti-gravity
    _fcM.pidConstants[pidIndex] = _sh.PIDS[pidIndex].getPID();
}

void FlightController::setPID_P_MSP(pid_index_e pidIndex, uint16_t kp)
{
    _sh.PIDS[pidIndex].setP(kp * _scaleFactors.kp);
    _fcM.pidConstants[pidIndex].kp = _sh.PIDS[pidIndex].getP();
}

void FlightController::setPID_I_MSP(pid_index_e pidIndex, uint16_t ki)
{
    _sh.PIDS[pidIndex].setI(ki * _scaleFactors.ki);
    _fcM.pidConstants[pidIndex].ki = _sh.PIDS[pidIndex].getI();
}

void FlightController::setPID_D_MSP(pid_index_e pidIndex, uint16_t kd)
{
    _sh.PIDS[pidIndex].setD(kd * _scaleFactors.kd);
    _fcM.pidConstants[pidIndex].kd = _sh.PIDS[pidIndex].getD();
}

void FlightController::setPID_F_MSP(pid_index_e pidIndex, uint16_t kf)
{
    _sh.PIDS[pidIndex].setF(kf * _scaleFactors.kf);
    _fcM.pidConstants[pidIndex].kf = _sh.PIDS[pidIndex].getF();
}

void FlightController::setPID_S_MSP(pid_index_e pidIndex, uint16_t ks)
{
    _sh.PIDS[pidIndex].setF(ks * _scaleFactors.ks);
    _fcM.pidConstants[pidIndex].ks = _sh.PIDS[pidIndex].getS();
}

uint32_t FlightController::getOutputPowerTimeMicroseconds() const
{
    //return _mixer.getOutputPowerTimeMicroseconds();
    return 0;
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
    _sh.takeOffCountStart = 0;
    _sh.groundMode = true;
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
    if (controlMode == _fcM.controlMode) {
        return;
    }
    _fcM.controlMode = controlMode;
    // reset the PID integral values when we change control mode
    for (auto& pid : _sh.PIDS) {
        pid.resetIntegral();
    }
}

void FlightController::setFiltersConfig(const filters_config_t& filtersConfig)
{
    const_cast<filters_config_t&>(_filtersConfig) = filtersConfig; // NOLINT(cppcoreguidelines-pro-type-const-cast)
    //!!TODO: check dT value for filters config
    const float dT = static_cast<float>(_taskIntervalMicroseconds) / 1000000.0F;
    //const float deltaT = (static_cast<float>(_ahrs.getTaskIntervalMicroseconds()) * 0.000001F) / static_cast<float>(_taskDenominator);

    if (filtersConfig.dterm_lpf1_hz == 0) {
        _sh.dTermFilters[ROLL_RATE_DPS].setToPassthrough();
        _sh.dTermFilters[ROLL_ANGLE_DEGREES].setToPassthrough();
        _sh.dTermFilters[ROLL_SIN_ANGLE].setToPassthrough();
        _sh.dTermFilters[PITCH_RATE_DPS].setToPassthrough();
        _sh.dTermFilters[PITCH_ANGLE_DEGREES].setToPassthrough();
        _sh.dTermFilters[PITCH_SIN_ANGLE].setToPassthrough();
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
            _sh.dTermFilters[ROLL_RATE_DPS].setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, dT);
            _sh.dTermFilters[ROLL_ANGLE_DEGREES].setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, dT);
            _sh.dTermFilters[ROLL_SIN_ANGLE].setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, dT);
            _sh.dTermFilters[PITCH_RATE_DPS].setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, dT);
            _sh.dTermFilters[PITCH_ANGLE_DEGREES].setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, dT);
            _sh.dTermFilters[PITCH_SIN_ANGLE].setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, dT);
            break;
        default:
            _sh.dTermFilters[ROLL_RATE_DPS].setToPassthrough();
            _sh.dTermFilters[ROLL_ANGLE_DEGREES].setToPassthrough();
            _sh.dTermFilters[ROLL_SIN_ANGLE].setToPassthrough();
            _sh.dTermFilters[PITCH_RATE_DPS].setToPassthrough();
            _sh.dTermFilters[PITCH_ANGLE_DEGREES].setToPassthrough();
            _sh.dTermFilters[PITCH_SIN_ANGLE].setToPassthrough();
            break;
        }
    }
    if (filtersConfig.output_lpf_hz == 0) {
        _sh.outputFilters[ROLL_RATE_DPS].setToPassthrough();
        _sh.outputFilters[PITCH_RATE_DPS].setToPassthrough();
        _sh.outputFilters[YAW_RATE_DPS].setToPassthrough();
    } else {
        const float ahrsDeltaT = static_cast<float>(_ahrs.getTaskIntervalMicroseconds()) * 0.000001F;
        _sh.outputFilters[ROLL_RATE_DPS].setCutoffFrequency(filtersConfig.output_lpf_hz, ahrsDeltaT);
        _sh.outputFilters[PITCH_RATE_DPS].setCutoffFrequency(filtersConfig.output_lpf_hz, ahrsDeltaT);
        _sh.outputFilters[YAW_RATE_DPS].setCutoffFrequency(filtersConfig.output_lpf_hz, ahrsDeltaT);
    }
}

void FlightController::setTPA_Config(const tpa_config_t& tpaConfig)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast)
    const_cast<tpa_config_t&>(_tpaConfig) = tpaConfig;

    // default of 1350 gives 0.35. range is limited to 0 to 0.99
    enum { PWM_RANGE_MIN = 1000 };
    const_cast<float&>(_tpa.breakpoint) = clip(static_cast<float>(tpaConfig.tpa_breakpoint - PWM_RANGE_MIN) / 1000.0F, 0.0F, 0.99F);
    const_cast<float&>(_tpa.multiplier) = (static_cast<float>(tpaConfig.tpa_rate) / 100.0F) / (1.0F - _tpa.breakpoint);

    // ensure tpaLowBreakpoint is always <= tpaBreakpoint
    const_cast<float&>(_tpa.lowBreakpoint) = std::fminf(clip((tpaConfig.tpa_low_breakpoint - PWM_RANGE_MIN) / 1000.0F, 0.01F, 1.0F), _tpa.breakpoint);
    // NOLINTEND(cppcoreguidelines-pro-type-const-cast)
}

void FlightController::setAntiGravityConfig(const anti_gravity_config_t& antiGravityConfig)
{
    // cast away constness to allow otherwise constant data to be set here
    // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast)
    const_cast<anti_gravity_config_t&>(_antiGravityConfig) = antiGravityConfig; 
    const_cast<anti_gravity_runtime_t&>(_antiGravity) = { 
        .PGain = static_cast<float>(antiGravityConfig.p_gain) * _scaleFactors.kp,
        .IGain = static_cast<float>(antiGravityConfig.i_gain) * _scaleFactors.ki
    };
    // NOLINTEND(cppcoreguidelines-pro-type-const-cast)
}

#if defined(USE_D_MAX)
void FlightController::setDMaxConfig(const d_max_config_t& dMaxConfig) // NOLINT(readability-make-member-function-const)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast)
    const_cast<d_max_config_t&>(_dMaxConfig) = dMaxConfig;
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        const uint8_t dMax = dMaxConfig.d_max[axis];
        const PIDF_uint16_t pid16 = getPID_Constants(static_cast<pid_index_e>(axis));
        if (pid16.kd > 0 && dMax > pid16.kd) {
            // ratio of DMax to kd, eg if kd is 8 and DMax is 10 then dMaxPercent is 1.25
            const_cast<float&>(_dMax.percent[axis]) = static_cast<float>(dMax) / pid16.kd;
        } else {
            const_cast<float&>(_dMax.percent[axis]) = 1.0F;
        }
    }
    const_cast<float&>(_dMax.gyroGain) = DMAX_GAIN_FACTOR * static_cast<float>(dMaxConfig.d_max_gain) / DMAX_LOWPASS_HZ;
    // lowpass included inversely in gain since stronger lowpass decreases peak effect
    const_cast<float&>(_dMax.setpointGain) = DMAX_SETPOINT_GAIN_FACTOR * static_cast<float>(dMaxConfig.d_max_gain * dMaxConfig.d_max_advance) / 100.0F / DMAX_LOWPASS_HZ;
    // NOLINTEND(cppcoreguidelines-pro-type-const-cast)
}
#endif

#if defined(USE_ITERM_RELAX)
void FlightController::setITermRelaxConfig(const iterm_relax_config_t& iTermRelaxConfig)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast)
    const_cast<iterm_relax_config_t&>(_iTermRelaxConfig) = iTermRelaxConfig;
    // NOLINTEND(cppcoreguidelines-pro-type-const-cast)
}
#endif

void FlightController::setCrashRecoveryConfig(const crash_recovery_config_t& crashRecoveryConfig)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-type-const-cast)
    const_cast<crash_recovery_config_t&>(_crashRecoveryConfig) = crashRecoveryConfig;
    const_cast<crash_recovery_runtime_t&>(_crash) = {
        .timeLimitUs = static_cast<uint32_t>(crashRecoveryConfig.crash_time * 1000),
        .timeDelayUs = static_cast<uint32_t>(crashRecoveryConfig.crash_delay * 1000),
        .recoveryAngleDeciDegrees = crashRecoveryConfig.crash_recovery_angle * 10,
        .recoveryRate = static_cast<float>(crashRecoveryConfig.crash_recovery_rate),
        .gyroThresholdDPS = static_cast<float>(crashRecoveryConfig.crash_gthreshold), // error in deg/s
        .DtermThresholdDPSPS = static_cast<float>(crashRecoveryConfig.crash_dthreshold * 1000), // gyro delta in deg/s/s * 1000 to match original 2017 intent
        .setpointThresholdDPS = static_cast<float>(crashRecoveryConfig.crash_setpoint_threshold),
        .limitYaw = static_cast<float>(crashRecoveryConfig.crash_limit_yaw)
    };
    // NOLINTEND(cppcoreguidelines-pro-type-const-cast)
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
        const PIDF::error_t rollRateError = _sh.PIDS[ROLL_RATE_DPS].getError();
        telemetry.rollRateError = { rollRateError.P, rollRateError.I, rollRateError.D, rollRateError.F, rollRateError.S };

        const PIDF::error_t pitchRateError = _sh.PIDS[PITCH_RATE_DPS].getError();
        telemetry.pitchRateError = { pitchRateError.P, pitchRateError.I, pitchRateError.D, pitchRateError.F, pitchRateError.S };

        const PIDF::error_t yawRateError = _sh.PIDS[YAW_RATE_DPS].getError();
        telemetry.yawRateError = { yawRateError.P, yawRateError.I, yawRateError.D, yawRateError.F, yawRateError.S };

        const PIDF::error_t rollAngleError = _sh.PIDS[ROLL_ANGLE_DEGREES].getError();
        telemetry.rollAngleError = { rollAngleError.P, rollAngleError.I, rollAngleError.D, rollAngleError.F, rollAngleError.S };

        const PIDF::error_t pitchAngleError = _sh.PIDS[PITCH_ANGLE_DEGREES].getError();
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
Called from within the VehicleControllerTask when signalled by the AHRS task that output data is available.
*/
void FlightController::outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem)
{
    ++_fcM.taskSignalledCount;
    if (_fcM.taskSignalledCount < _taskDenominator) {
        return;
    }
    _fcM.taskSignalledCount = 0;

    if (_radioController.getFailsafePhase() == RadioController::FAILSAFE_RX_LOSS_DETECTED) {
        MotorMixerBase::commands_t commands {
            .throttle  = 0.25F,
            .roll   = 0.0F,
            .pitch  = 0.0F,
            .yaw    = 0.0F
        };
        _mixer.outputToMotors(commands, deltaT, tickCount);
        // the mixer may adjust the throttle value, so save this value for the blackbox record
        _fcM.mixerAdjustedThrottle= commands.throttle;
        return;
    }

    MotorMixerBase::commands_t commands {
        .throttle  = queueItem.throttle,
        // scale roll, pitch, and yaw to range [0.0F, 1.0F]
        .roll   = queueItem.roll / _rollRateAtMaxPowerDPS,
        .pitch  = queueItem.pitch / _pitchRateAtMaxPowerDPS,
        .yaw    = queueItem.yaw / _yawRateAtMaxPowerDPS
    };

    _mixer.outputToMotors(commands, deltaT, tickCount);
    // the mixer may adjust the throttle value, so save this value for the blackbox record
    _fcM.mixerAdjustedThrottle= commands.throttle;
}
