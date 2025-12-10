#include "FlightController.h"

#include <Debug.h>

#if !defined(FRAMEWORK_TEST)
//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif
#endif

#include <MotorMixerBase.h>
#include <TimeMicroseconds.h>

#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <esp32-hal.h>
#endif


/*!
Constructor. Sets member data.

The FlightController task has the same task interval as the AHRS task.
The FlightController task function outputToMixer waits on a message queue that is signalled once every time
the AHRS task function AHRS::readIMUandUpdateOrientation runs.

MotorMixer::outputToMotors is called every _outputToMotorsDenominator times FlightController outputToMixer is called
*/
FlightController::FlightController(uint32_t taskIntervalMicroseconds, uint32_t outputToMotorsDenominator, MotorMixerBase& motorMixer, AHRS_MessageQueue& ahrsMessageQueue, Debug& debug) :
    VehicleControllerBase(AIRCRAFT, PID_COUNT, taskIntervalMicroseconds),
    _motorMixer(motorMixer),
    _ahrsMessageQueue(ahrsMessageQueue),
    _debug(debug),
    _outputToMotorsDenominator(outputToMotorsDenominator),
    _fcC(_fcM),
    _rxC(_rxM)
{
    _sh.antiGravityThrottleFilter.setToPassthrough();
}
// NOLINTBEGIN(cppcoreguidelines-macro-usage,bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
// #defines to catch inadvertent use of _rxM or _ahM in this file.
#define _rxM "error not modifiable in this task"
#define _ahM "error not modifiable in this task"
// NOLINTEND(cppcoreguidelines-macro-usage,bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)

static const std::array<std::string, FlightController::PID_COUNT> PID_NAMES = {
    "ROLL_RATE",
    "PITCH_RATE",
    "YAW_RATE",
    "ROLL_ANGLE",
    "PITCH_ANGLE",
#if defined(USE_SIN_ANGLE_PIDS)
    "ROLL_SIN_ANGLE",
    "PITCH_SIN_ANGLE"
#endif
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
        .ks = static_cast<uint16_t>(_sh.PIDS[pidIndex].getS() / _scaleFactors.ks),
        .kk = static_cast<uint16_t>(_sh.PIDS[pidIndex].getK() / _scaleFactors.kk),
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
        static_cast<uint16_t>(std::lroundf(pid.ks / _scaleFactors.ks)),
        static_cast<uint16_t>(std::lroundf(pid.kk / _scaleFactors.kk))
    };

    return pid16;
}

/*!
Set the P, I, D, S, and K values for the PID with index pidIndex.
Integration is switched off, so that there is no integral windup before takeoff.
*/
void FlightController::setPID_Constants(pid_index_e pidIndex, const PIDF_uint16_t& pid16)
{

    const PIDF::PIDF_t pid = {
        static_cast<float>(pid16.kp) * _scaleFactors.kp,
        static_cast<float>(pid16.ki) * _scaleFactors.ki,
        static_cast<float>(pid16.kd) * _scaleFactors.kd,
        static_cast<float>(pid16.ks) * _scaleFactors.ks,
        static_cast<float>(pid16.kk) * _scaleFactors.kk
    };

    _sh.PIDS[pidIndex].setPID(pid);
    _sh.PIDS[pidIndex].switchIntegrationOff();
    // keep copies of the PID constants so they can be adjusted by anti-gravity
    _fcM.pidConstants[pidIndex] = _sh.PIDS[pidIndex].getPID();
}

void FlightController::setPID_P_MSP(pid_index_e pidIndex, uint16_t kp)
{
    _sh.PIDS[pidIndex].setP(kp * _scaleFactors.kp);
    _fcM.pidConstants[pidIndex].kp = _sh.PIDS[pidIndex].getP();
}

void FlightController::setPID_PD_MSP(pid_index_e pidIndex, uint16_t kp)
{
    const PIDF pid = getPID(pidIndex);
    const float ratio = pid.getD() / pid.getP();
    setPID_P_MSP(pidIndex, kp);
    setPID_D_MSP(pidIndex, static_cast<uint16_t>(static_cast<float>(kp)*ratio));
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

void FlightController::setPID_S_MSP(pid_index_e pidIndex, uint16_t ks)
{
    _sh.PIDS[pidIndex].setS(ks * _scaleFactors.ks);
    _fcM.pidConstants[pidIndex].ks = _sh.PIDS[pidIndex].getS();
}

void FlightController::setPID_K_MSP(pid_index_e pidIndex, uint16_t kk)
{
    _sh.PIDS[pidIndex].setK(kk * _scaleFactors.kk);
    _fcM.pidConstants[pidIndex].kk = _sh.PIDS[pidIndex].getK();
}

const FlightController::simplified_pid_settings_t& FlightController::getSimplifiedPID_Settings() const
{
    return _fcM.simplifiedPID_Settings;
}

void FlightController::setSimplifiedPID_Settings(const simplified_pid_settings_t& settings)
{
    _fcM.simplifiedPID_Settings = settings;
    const float masterMultiplier = static_cast<float>(settings.multiplier) / 100.0F;
    const float piGain = static_cast<float>(settings.pi_gain) / 100.0F;
    const float dGain = static_cast<float>(settings.d_gain) / 100.0F;
    const float kGain = static_cast<float>(settings.k_gain) / 100.0F;
    const float iGain = static_cast<float>(settings.i_gain) / 100.0F;

    static constexpr float PID_GAIN_MAX = 250.0F;
    static constexpr float K_GAIN_MAX = 1000.0F;
    PIDF_uint16_t pid16 {};

    pid16.kp = static_cast<uint16_t>(std::clamp(DefaultPIDs[ROLL_RATE_DPS].kp * masterMultiplier * piGain, 0.0F, PID_GAIN_MAX));
    pid16.ki = static_cast<uint16_t>(std::clamp(DefaultPIDs[ROLL_RATE_DPS].ki * masterMultiplier * piGain * iGain, 0.0F, PID_GAIN_MAX));
    pid16.kd = static_cast<uint16_t>(std::clamp(DefaultPIDs[ROLL_RATE_DPS].kd * masterMultiplier * dGain, 0.0F, PID_GAIN_MAX));
    pid16.kk = static_cast<uint16_t>(std::clamp(DefaultPIDs[ROLL_RATE_DPS].kk * masterMultiplier * kGain, 0.0F, K_GAIN_MAX));
    setPID_Constants(ROLL_RATE_DPS, pid16);

    pid16.kp = static_cast<uint16_t>(std::clamp(DefaultPIDs[PITCH_RATE_DPS].kp * masterMultiplier * piGain, 0.0F, PID_GAIN_MAX));
    const float pitchPI_gain = static_cast<float>(settings.pitch_pi_gain) / 100.0F;
    pid16.ki = static_cast<uint16_t>(std::clamp(DefaultPIDs[PITCH_RATE_DPS].ki * masterMultiplier * piGain * iGain * pitchPI_gain, 0.0F, PID_GAIN_MAX));
    const float pitchDGain = static_cast<float>(settings.roll_pitch_ratio) / 100.0F;
    pid16.kd = static_cast<uint16_t>(std::clamp(DefaultPIDs[PITCH_RATE_DPS].kd * masterMultiplier * dGain * pitchDGain, 0.0F, PID_GAIN_MAX));
    pid16.kk = static_cast<uint16_t>(std::clamp(DefaultPIDs[PITCH_RATE_DPS].kk * masterMultiplier * kGain, 0.0F, K_GAIN_MAX));
    setPID_Constants(PITCH_RATE_DPS, pid16);

    if (_pidTuningMode == PID_TUNING_SIMPLIFIED_RPY) {
        pid16.kp = static_cast<uint16_t>(std::clamp(DefaultPIDs[YAW_RATE_DPS].kp * masterMultiplier * piGain, 0.0F, PID_GAIN_MAX));
        pid16.ki = static_cast<uint16_t>(std::clamp(DefaultPIDs[YAW_RATE_DPS].ki * masterMultiplier * piGain * iGain, 0.0F, PID_GAIN_MAX));
        pid16.kd = static_cast<uint16_t>(std::clamp(DefaultPIDs[YAW_RATE_DPS].kd * masterMultiplier * dGain, 0.0F, PID_GAIN_MAX));
        pid16.kk = static_cast<uint16_t>(std::clamp(DefaultPIDs[YAW_RATE_DPS].kk * masterMultiplier * kGain, 0.0F, K_GAIN_MAX));
        setPID_Constants(YAW_RATE_DPS, pid16);
    }

#if defined(USE_D_MAX)
    const float dMaxGainRoll = static_cast<float>(settings.d_max_gain + (100 - settings.d_max_gain) * DefaultPIDs[ROLL_RATE_DPS].kd) / (100.0F*D_MAX_DEFAULT_ROLL);
    _dMaxConfig.d_max[ROLL_RATE_DPS] = static_cast<uint8_t>(std::clamp(D_MAX_DEFAULT_ROLL * masterMultiplier * dGain * dMaxGainRoll, 0.0F, PID_GAIN_MAX));

    const float dMaxGainPitch = static_cast<float>(settings.d_max_gain + (100 - settings.d_max_gain) * DefaultPIDs[PITCH_RATE_DPS].kd) / (100.0F*D_MAX_DEFAULT_PITCH);
    _dMaxConfig.d_max[PITCH_RATE_DPS] = static_cast<uint8_t>(std::clamp(D_MAX_DEFAULT_PITCH * masterMultiplier * dGain * pitchDGain * dMaxGainPitch, 0.0F, PID_GAIN_MAX));
#endif
}

uint32_t FlightController::getOutputPowerTimeMicroseconds() const
{
    //return _motorMixer.getOutputPowerTimeMicroseconds();
    return 0;
}

void FlightController::motorsSwitchOff()
{
    _motorMixer.motorsSwitchOff();
    _sh.takeOffCountStart = 0;
    _sh.groundMode = true;
    switchPID_integrationOff();
}

void FlightController::motorsSwitchOn()
{
    // don't allow motors to be switched on if the sensor fusion has not initialized
    //if (!_sensorFusionFilterIsInitializing) {
        _motorMixer.motorsSwitchOn();
        // reset the PID integral values when we switch the motors on
        switchPID_integrationOn();
    //}
}

bool FlightController::motorsIsDisabled() const
{
    return _motorMixer.motorsIsDisabled();
}

bool FlightController::motorsIsOn() const
{
    return _motorMixer.motorsIsOn();
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

void FlightController::setPID_TuningMode(pid_tuning_mode_e pidTuningMode)
{
    _pidTuningMode = pidTuningMode;
}

void FlightController::setFiltersConfig(const filters_config_t& filtersConfig)
{
    _filtersConfig = filtersConfig;

    // always used PT1 filters for DTerm filters.
#if defined(USE_DTERM_FILTERS_EXTENDED)
    _filtersConfig.dterm_lpf1_type = FlightController::filters_config_t::PT1;
    _filtersConfig.dterm_lpf2_type = FlightController::filters_config_t::PT1;
#endif

    //!!TODO: check dT value for filters config
    const float deltaT = static_cast<float>(_taskIntervalMicroseconds) * 0.000001F;
    //const float deltaT = (static_cast<float>(_ahrs.getTaskIntervalMicroseconds()) * 0.000001F) / static_cast<float>(_outputToMotorsDenominator);

    // DTerm filters
    if (_filtersConfig.dterm_lpf1_hz == 0) {
        for (auto& filter : _sh.dTermFilters1) {
            filter.setToPassthrough();
        }
    } else {
        // DTerm filters always use a PowerTransfer1 filter.
        for (auto& filter : _sh.dTermFilters1) {
            filter.setCutoffFrequencyAndReset(filtersConfig.dterm_lpf1_hz, deltaT);
        }
    }
    if (_filtersConfig.dterm_lpf2_hz == 0) {
        for (auto& filter : _sh.dTermFilters2) {
            filter.setToPassthrough();
        }
    } else {
        // DTerm filters always use a PowerTransfer1 filter.
        for (auto& filter : _sh.dTermFilters2) {
            filter.setCutoffFrequencyAndReset(filtersConfig.dterm_lpf2_hz, deltaT);
        }
    }

    // Output filters
    if (_filtersConfig.output_lpf_hz == 0) {
        for (auto& filter : _fcM.outputFilters) {
            filter.setToPassthrough();
        }
    } else {
        for (auto& filter : _fcM.outputFilters) {
            filter.setCutoffFrequencyAndReset(filtersConfig.output_lpf_hz, deltaT);
        }
    }
}

void FlightController::setFlightModeConfig(const flight_mode_config_t& flightModeConfig)
{
    _flightModeConfig = flightModeConfig;
}

void FlightController::setTPA_Config(const tpa_config_t& tpaConfig)
{
    _tpaConfig = tpaConfig;

    // default of 1350 gives 0.35. range is limited to 0 to 0.99
    enum { PWM_RANGE_MIN = 1000 };
    _tpa.breakpoint = std::clamp(static_cast<float>(tpaConfig.tpa_breakpoint - PWM_RANGE_MIN) / 1000.0F, 0.0F, 0.99F);
    _tpa.multiplier = (static_cast<float>(tpaConfig.tpa_rate) / 100.0F) / (1.0F - _tpa.breakpoint);

    // ensure tpaLowBreakpoint is always <= tpaBreakpoint
    _tpa.lowBreakpoint = std::fminf(std::clamp(static_cast<float>(tpaConfig.tpa_low_breakpoint - PWM_RANGE_MIN) / 1000.0F, 0.01F, 1.0F), _tpa.breakpoint);
}

void FlightController::setAntiGravityConfig(const anti_gravity_config_t& antiGravityConfig)
{
    _antiGravityConfig = antiGravityConfig; 
    _antiGravity = { 
        .PGain = static_cast<float>(antiGravityConfig.p_gain) * _scaleFactors.kp,
        .IGain = static_cast<float>(antiGravityConfig.i_gain) * _scaleFactors.ki
    };
}

#if defined(USE_D_MAX)
void FlightController::setDMaxConfig(const d_max_config_t& dMaxConfig) // NOLINT(readability-make-member-function-const)
{
    _dMaxConfig = dMaxConfig;
    for (size_t axis = 0; axis < RPY_AXIS_COUNT; ++axis) {
        const uint8_t dMax = dMaxConfig.d_max[axis];
        const PIDF_uint16_t pid16 = getPID_Constants(static_cast<pid_index_e>(axis));
        if (pid16.kd > 0 && dMax > pid16.kd) {
            // ratio of DMax to kd, eg if kd is 8 and DMax is 10 then dMaxPercent is 1.25
            _dMax.percent[axis] = static_cast<float>(dMax) / pid16.kd;
        } else {
            _dMax.percent[axis] = 1.0F;
        }
    }
    _dMax.gyroGain = DMAX_GAIN_FACTOR * static_cast<float>(dMaxConfig.d_max_gain) / DMAX_LOWPASS_HZ;
    // lowpass included inversely in gain since stronger lowpass decreases peak effect
    _dMax.setpointGain = DMAX_SETPOINT_GAIN_FACTOR * static_cast<float>(dMaxConfig.d_max_gain * dMaxConfig.d_max_advance) / 100.0F / DMAX_LOWPASS_HZ;
    for (auto& filter : _sh.dMaxRangeFilters) {
        filter.setToPassthrough();
    }
    for (auto& filter : _sh.dMaxLowpassFilters) {
        filter.setToPassthrough();
    }
}
#endif

#if defined(USE_ITERM_RELAX)
void FlightController::setITermRelaxConfig(const iterm_relax_config_t& iTermRelaxConfig)
{
    _iTermRelaxConfig = iTermRelaxConfig;
    _iTermRelax.setpointThresholdDPS = iTermRelaxConfig.iterm_relax_setpoint_threshold;
    for (auto& filter : _sh.iTermRelaxFilters) {
        filter.setToPassthrough();
    }
}
#endif

#if defined(USE_YAW_SPIN_RECOVERY)
void FlightController::setYawSpinRecoveryConfig(const yaw_spin_recovery_config_t& yawSpinRecoveryConfig)
{
    _yawSpinRecoveryConfig = yawSpinRecoveryConfig;
}
#endif

#if defined(USE_CRASH_RECOVERY)
void FlightController::setCrashRecoveryConfig(const crash_recovery_config_t& crashRecoveryConfig)
{
    _crashRecoveryConfig = crashRecoveryConfig;
    _crash = {
        .timeLimitUs = static_cast<uint32_t>(crashRecoveryConfig.crash_time * 1000),
        .timeDelayUs = static_cast<uint32_t>(crashRecoveryConfig.crash_delay * 1000),
        .recoveryAngleDeciDegrees = crashRecoveryConfig.crash_recovery_angle * 10,
        .recoveryRate = static_cast<float>(crashRecoveryConfig.crash_recovery_rate),
        .gyroThresholdDPS = static_cast<float>(crashRecoveryConfig.crash_gthreshold), // error in deg/s
        .DtermThresholdDPSPS = static_cast<float>(crashRecoveryConfig.crash_dthreshold * 1000), // gyro delta in deg/s/s * 1000 to match original 2017 intent
        .setpointThresholdDPS = static_cast<float>(crashRecoveryConfig.crash_setpoint_threshold),
        .limitYaw = static_cast<float>(crashRecoveryConfig.crash_limit_yaw)
    };
}
#endif

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

    for (size_t ii = 0; ii < _motorMixer.getMotorCount(); ++ ii) {
        telemetry.motors[ii].power = _motorMixer.getMotorOutput(ii);
        telemetry.motors[ii].rpm = _motorMixer.getMotorRPM(ii);
    }
    if (motorsIsOn()) {
        const PIDF::error_t rollRateError = _sh.PIDS[ROLL_RATE_DPS].getError();
        telemetry.rollRateError = { rollRateError.P, rollRateError.I, rollRateError.D, rollRateError.S, rollRateError.K };

        const PIDF::error_t pitchRateError = _sh.PIDS[PITCH_RATE_DPS].getError();
        telemetry.pitchRateError = { pitchRateError.P, pitchRateError.I, pitchRateError.D, pitchRateError.S, pitchRateError.K };

        const PIDF::error_t yawRateError = _sh.PIDS[YAW_RATE_DPS].getError();
        telemetry.yawRateError = { yawRateError.P, yawRateError.I, yawRateError.D, yawRateError.S, yawRateError.K };

        const PIDF::error_t rollAngleError = _sh.PIDS[ROLL_ANGLE_DEGREES].getError();
        telemetry.rollAngleError = { rollAngleError.P, rollAngleError.I, rollAngleError.D, rollAngleError.S, rollAngleError.K };

        const PIDF::error_t pitchAngleError = _sh.PIDS[PITCH_ANGLE_DEGREES].getError();
        telemetry.pitchAngleError = { pitchAngleError.P, pitchAngleError.I, pitchAngleError.D, pitchAngleError.S, pitchAngleError.K };
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
NOTE: CALLED FROM WITHIN THE FlightController TASK
It is typically called at frequency of between 1000Hz and 8000Hz, so it has to be FAST.

Called by the scheduler when signalled by the AHRS task that output data is available.
*/
void FlightController::outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem)
{
    // Filter the output.
    // This smooths the output, but also accumulates the output in the filter,
    // so the values influence the output even when `outputToMotors` is not called.
    _fcM.outputs[FD_ROLL] = _fcM.outputFilters[FD_ROLL].filter(queueItem.roll);
    _fcM.outputs[FD_PITCH] = _fcM.outputFilters[FD_PITCH].filter(queueItem.pitch);
    _fcM.outputs[FD_YAW] = _fcM.outputFilters[FD_YAW].filter(queueItem.yaw);

    // Output to motors every _outputToMotorsDenominator times outputToMixer is called.
    ++_fcM.outputToMixerCount;
    if (_fcM.outputToMixerCount >= _outputToMotorsDenominator) {
        _fcM.outputToMixerCount = 0;

        MotorMixerBase::commands_t commands {
            .throttle  = queueItem.throttle,
            // scale roll, pitch, and yaw to range [0.0F, 1.0F]
            .roll   = _fcM.outputs[FD_ROLL] / _rollRateAtMaxPowerDPS,
            .pitch  = _fcM.outputs[FD_PITCH] / _pitchRateAtMaxPowerDPS,
            .yaw    = _fcM.outputs[FD_YAW] / _yawRateAtMaxPowerDPS
        };

        _motorMixer.outputToMotors(commands, deltaT, tickCount);
    }

#if defined(USE_RPM_FILTERS)
    // perform an RPM filter iteration step, even if we have not output to the motors
    _motorMixer.rpmFilterSetFrequencyHzIterationStep();
#endif
}
