#include "FlightController.h"

#include <debug.h>

#if !defined(FRAMEWORK_TEST)
//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif
#endif

#include <motor_mixer_base.h>
#include <time_microseconds.h>

#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <esp32-hal.h>
#endif

#if (__cplusplus >= 202002L)
#include <ranges>
#endif


/*!
Constructor. Sets member data.

The FlightController task has the same task interval as the AHRS task.
The FlightController task function outputToMixer waits on a message queue that is signalled once every time
the AHRS task function AHRS::readIMUandUpdateOrientation runs.

MotorMixer::outputToMotors is called every _outputToMotorsDenominator times FlightController outputToMixer is called
*/
FlightController::FlightController(uint32_t task_interval_microseconds) :
    VehicleControllerBase(AIRCRAFT, PID_COUNT, task_interval_microseconds)
{
    _sh.antiGravityThrottleFilter.set_to_passthrough();

    static constexpr float outputSaturationValue = 1.0F / MIXER_OUTPUT_SCALE_FACTOR;
    _sh.PIDS[ROLL_RATE_DPS].set_output_saturation_value(outputSaturationValue);
    _sh.PIDS[PITCH_RATE_DPS].set_output_saturation_value(outputSaturationValue);
    _sh.PIDS[YAW_RATE_DPS].set_output_saturation_value(outputSaturationValue);
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

const std::string& FlightController::getPID_Name(pid_index_e pid_index) const
{
    return PID_NAMES[pid_index];
}

VehicleControllerBase::PIDF_error_t FlightController::get_pid_error(size_t index) const
{
    const pid_error_t error = _sh.PIDS[index].get_error();
    return PIDF_error_t {
        .P = error.p,
        .I = error.i,
        .D = error.d,
        .S = error.s,
        .K = error.k
    };
}

float FlightController::get_pid_setpoint(size_t index) const
{
    return _sh.PIDS[index].get_setpoint();
}

//!!TODO: reconcile FlightController::get_pid_msp, FlightController::get_pid_constants and set variants
VehicleControllerBase::PIDF_uint16_t FlightController::get_pid_msp(size_t index) const
{
    assert(index < PID_COUNT);

    const auto pid_index = static_cast<pid_index_e>(index);
    const PIDF_uint16_t ret = {
        .kp = static_cast<uint16_t>(_sh.PIDS[pid_index].get_p() / _scaleFactors.kp),
        .ki = static_cast<uint16_t>(_sh.PIDS[pid_index].get_i() / _scaleFactors.ki),
        .kd = static_cast<uint16_t>(_sh.PIDS[pid_index].get_d() / _scaleFactors.kd),
        .ks = static_cast<uint16_t>(_sh.PIDS[pid_index].get_s() / _scaleFactors.ks),
        .kk = static_cast<uint16_t>(_sh.PIDS[pid_index].get_k() / _scaleFactors.kk),
    };
    return ret;
}

VehicleControllerBase::PIDF_uint16_t FlightController::get_pid_constants(pid_index_e pid_index) const
{
    const pid_constants_t pid = _sh.PIDS[pid_index].get_pid();
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
Set the P, I, D, S, and K values for the PID with index pid_index.
Integration is switched off, so that there is no integral windup before takeoff.
*/
void FlightController::set_pid_constants(pid_index_e pid_index, const PIDF_uint16_t& pid16)
{

    const pid_constants_t pid = {
        static_cast<float>(pid16.kp) * _scaleFactors.kp,
        static_cast<float>(pid16.ki) * _scaleFactors.ki,
        static_cast<float>(pid16.kd) * _scaleFactors.kd,
        static_cast<float>(pid16.ks) * _scaleFactors.ks,
        static_cast<float>(pid16.kk) * _scaleFactors.kk
    };

    _sh.PIDS[pid_index].set_pid(pid);
    _sh.PIDS[pid_index].switch_integration_off();
    _sh.PIDS[pid_index].set_setpoint(0.0F);
    // keep copies of the PID constants so they can be adjusted by anti-gravity
    _fcM.pidConstants[pid_index] = _sh.PIDS[pid_index].get_pid();
}

void FlightController::set_pid_p_msp(pid_index_e pid_index, uint16_t kp)
{
    _sh.PIDS[pid_index].set_p(kp * _scaleFactors.kp);
    _fcM.pidConstants[pid_index].kp = _sh.PIDS[pid_index].get_p();
}

void FlightController::set_pid_pd_msp(pid_index_e pid_index, uint16_t kp)
{
    const PidController pid = getPID(pid_index);
    const float ratio = pid.get_d() / pid.get_p();
    set_pid_p_msp(pid_index, kp);
    set_pid_d_msp(pid_index, static_cast<uint16_t>(static_cast<float>(kp)*ratio));
}

void FlightController::set_pid_i_msp(pid_index_e pid_index, uint16_t ki)
{
    _sh.PIDS[pid_index].set_i(ki * _scaleFactors.ki);
    _fcM.pidConstants[pid_index].ki = _sh.PIDS[pid_index].get_i();
}

void FlightController::set_pid_d_msp(pid_index_e pid_index, uint16_t kd)
{
    _sh.PIDS[pid_index].set_d(kd * _scaleFactors.kd);
    _fcM.pidConstants[pid_index].kd = _sh.PIDS[pid_index].get_d();
}

void FlightController::set_pid_s_msp(pid_index_e pid_index, uint16_t ks)
{
    _sh.PIDS[pid_index].set_s(ks * _scaleFactors.ks);
    _fcM.pidConstants[pid_index].ks = _sh.PIDS[pid_index].get_s();
}

void FlightController::set_pid_k_msp(pid_index_e pid_index, uint16_t kk)
{
    _sh.PIDS[pid_index].set_k(kk * _scaleFactors.kk);
    _fcM.pidConstants[pid_index].kk = _sh.PIDS[pid_index].get_k();
}

const simplified_pid_settings_t& FlightController::getSimplifiedPID_Settings() const
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

    PIDF_uint16_t pid16 {};

    pid16.kp = std::clamp(static_cast<uint16_t>(DefaultPIDs[ROLL_RATE_DPS].kp * masterMultiplier * piGain), uint16_t{0}, PID_GAIN_MAX);
    pid16.ki = std::clamp(static_cast<uint16_t>(DefaultPIDs[ROLL_RATE_DPS].ki * masterMultiplier * piGain * iGain), uint16_t{0}, PID_GAIN_MAX);
    pid16.kd = std::clamp(static_cast<uint16_t>(DefaultPIDs[ROLL_RATE_DPS].kd * masterMultiplier * dGain), uint16_t{0}, PID_GAIN_MAX);
    pid16.kk = std::clamp(static_cast<uint16_t>(DefaultPIDs[ROLL_RATE_DPS].kk * masterMultiplier * kGain), uint16_t{0}, K_GAIN_MAX);
    set_pid_constants(ROLL_RATE_DPS, pid16);

    pid16.kp = std::clamp(static_cast<uint16_t>(DefaultPIDs[PITCH_RATE_DPS].kp * masterMultiplier * piGain), uint16_t{0}, PID_GAIN_MAX);
    const float pitchPI_gain = static_cast<float>(settings.pitch_pi_gain) / 100.0F;
    pid16.ki = std::clamp(static_cast<uint16_t>(DefaultPIDs[PITCH_RATE_DPS].ki * masterMultiplier * piGain * iGain * pitchPI_gain), uint16_t{0}, PID_GAIN_MAX);
    const float pitchDGain = static_cast<float>(settings.roll_pitch_ratio) / 100.0F;
    pid16.kd = std::clamp(static_cast<uint16_t>(DefaultPIDs[PITCH_RATE_DPS].kd * masterMultiplier * dGain * pitchDGain), uint16_t{0}, PID_GAIN_MAX);
    pid16.kk = std::clamp(static_cast<uint16_t>(DefaultPIDs[PITCH_RATE_DPS].kk * masterMultiplier * kGain), uint16_t{0}, K_GAIN_MAX);
    set_pid_constants(PITCH_RATE_DPS, pid16);

    if (_pidTuningMode == PID_TUNING_SIMPLIFIED_RPY) {
        pid16.kp = std::clamp(static_cast<uint16_t>(DefaultPIDs[YAW_RATE_DPS].kp * masterMultiplier * piGain), uint16_t{0}, PID_GAIN_MAX);
        pid16.ki = std::clamp(static_cast<uint16_t>(DefaultPIDs[YAW_RATE_DPS].ki * masterMultiplier * piGain * iGain), uint16_t{0}, PID_GAIN_MAX);
        pid16.kd = std::clamp(static_cast<uint16_t>(DefaultPIDs[YAW_RATE_DPS].kd * masterMultiplier * dGain), uint16_t{0}, PID_GAIN_MAX);
        pid16.kk = std::clamp(static_cast<uint16_t>(DefaultPIDs[YAW_RATE_DPS].kk * masterMultiplier * kGain), uint16_t{0}, K_GAIN_MAX);
        set_pid_constants(YAW_RATE_DPS, pid16);
    }

#if defined(USE_D_MAX)
    const float dMaxGainRoll = static_cast<float>(settings.d_max_gain + (100 - settings.d_max_gain) * DefaultPIDs[ROLL_RATE_DPS].kd) / (100.0F*D_MAX_DEFAULT_ROLL);
    _dMaxConfig.d_max[ROLL_RATE_DPS] = std::clamp(static_cast<uint8_t>(D_MAX_DEFAULT_ROLL * masterMultiplier * dGain * dMaxGainRoll), uint8_t{0}, uint8_t{PID_GAIN_MAX});

    const float dMaxGainPitch = static_cast<float>(settings.d_max_gain + (100 - settings.d_max_gain) * DefaultPIDs[PITCH_RATE_DPS].kd) / (100.0F*D_MAX_DEFAULT_PITCH);
    _dMaxConfig.d_max[PITCH_RATE_DPS] = std::clamp(static_cast<uint8_t>(D_MAX_DEFAULT_PITCH * masterMultiplier * dGain * pitchDGain * dMaxGainPitch), uint8_t{0}, uint8_t{PID_GAIN_MAX});
#endif
}

uint32_t FlightController::get_output_power_time_microseconds() const
{
    //return _motorMixer.get_output_power_time_microseconds();
    return 0;
}

void FlightController::motorsSwitchOff(MotorMixerBase& motorMixer)
{
    motorMixer.motors_switch_off();
    _sh.takeOffCountStart = 0;
    _sh.groundMode = true;
    switchPID_integrationOff();
}

void FlightController::motorsSwitchOn(MotorMixerBase& motorMixer)
{
    // don't allow motors to be switched on if the sensor fusion has not initialized
#if defined(FRAMEWORK_TEST)
    if (!_sensor_fusion_filter_is_initializing) { //!!TODO: fix _sensorFusionFilterIsInitializing
#else
    {
#endif
        motorMixer.motors_switch_on();
        // reset the PID integral values when we switch the motors on
        switchPID_integrationOn();
    }
}

void FlightController::setBlackboxActive(bool isActive) 
{
    _sh.blackboxActive = isActive;
}

void FlightController::set_pid_tuning_mode(pid_tuning_mode_e pidTuningMode)
{
    _pidTuningMode = pidTuningMode;
}

void FlightController::setMaxAngleRates(float maxRollRateDPS, float maxPitchRateDPS, float maxYawRateDPS)
{
    _maxRollRateDPS = maxRollRateDPS;
    _maxPitchRateDPS = maxPitchRateDPS;
    _maxYawRateDPS = maxYawRateDPS;
}

void FlightController::setFiltersConfig(const flight_controller_filters_config_t& filtersConfig)
{
    _filtersConfig = filtersConfig;

    // always used PT1 filters for DTerm filters.
#if defined(USE_DTERM_FILTERS_EXTENDED)
    _filtersConfig.dterm_lpf1_type = flight_controller_filters_config_t::PT1;
    _filtersConfig.dterm_lpf2_type = flight_controller_filters_config_t::PT1;
#endif

    //!!TODO: check dT value for filters config
    const float deltaT = static_cast<float>(_task_interval_microseconds) * 0.000001F;
    //const float deltaT = (static_cast<float>(_ahrs.get_task_interval_microseconds()) * 0.000001F) / static_cast<float>(_outputToMotorsDenominator);

    // DTerm filters
    if (_filtersConfig.dterm_lpf1_hz == 0) {
        for (auto& filter : _sh.dTermFilters1) {
            filter.set_to_passthrough();
        }
    } else {
        // DTerm filters always use a PowerTransfer1 filter.
        for (auto& filter : _sh.dTermFilters1) {
            filter.set_cutoff_frequency_and_reset(filtersConfig.dterm_lpf1_hz, deltaT);
        }
    }
    if (_filtersConfig.dterm_lpf2_hz == 0) {
        for (auto& filter : _sh.dTermFilters2) {
            filter.set_to_passthrough();
        }
    } else {
        // DTerm filters always use a PowerTransfer1 filter.
        for (auto& filter : _sh.dTermFilters2) {
            filter.set_cutoff_frequency_and_reset(filtersConfig.dterm_lpf2_hz, deltaT);
        }
    }

    // Output filters
    if (_filtersConfig.output_lpf_hz == 0) {
        for (auto& filter : _sh.outputFilters) {
            filter.set_to_passthrough();
        }
    } else {
        for (auto& filter : _sh.outputFilters) {
            filter.set_cutoff_frequency_and_reset(filtersConfig.output_lpf_hz, deltaT);
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

void FlightController::setCrashFlipConfig(const crash_flip_config_t& crashFlipConfig)
{
    _crashFlipConfig = crashFlipConfig;
}

#if defined(USE_D_MAX)
void FlightController::setDMaxConfig(const d_max_config_t& dMaxConfig) // NOLINT(readability-make-member-function-const)
{
    _dMaxConfig = dMaxConfig;
#if (__cplusplus >= 202002L)
    for (auto axis : std::views::iota(size_t{0}, size_t{RPY_AXIS_COUNT})) {
#else
    for (size_t axis = 0; axis < RPY_AXIS_COUNT; ++axis) {
#endif
        const uint8_t dMax = dMaxConfig.d_max[axis];
        const PIDF_uint16_t pid16 = get_pid_constants(static_cast<pid_index_e>(axis));
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
        filter.set_to_passthrough();
    }
    for (auto& filter : _sh.dMaxLowpassFilters) {
        filter.set_to_passthrough();
    }
}
#endif

#if defined(USE_ITERM_RELAX)
void FlightController::setITermRelaxConfig(const iterm_relax_config_t& iTermRelaxConfig)
{
    _iTermRelaxConfig = iTermRelaxConfig;
    _iTermRelax.setpointThresholdDPS = iTermRelaxConfig.iterm_relax_setpoint_threshold;
    for (auto& filter : _sh.iTermRelaxFilters) {
        filter.set_to_passthrough();
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
flight_controller_quadcopter_telemetry_t FlightController::getTelemetryData(const MotorMixerBase& motorMixer) const
{
    flight_controller_quadcopter_telemetry_t telemetry;

#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{motorMixer.get_motor_count()})) {
#else
    for (size_t ii = 0; ii < motorMixer.get_motor_count(); ++ ii) {
#endif
        telemetry.motors[ii].power = motorMixer.get_motor_output(ii);
        telemetry.motors[ii].rpm = motorMixer.get_motor_rpm(ii);
    }
    if (motorMixer.motors_is_on()) {
        const pid_error_t rollRateError = _sh.PIDS[ROLL_RATE_DPS].get_error();
        telemetry.rollRateError = { rollRateError.p, rollRateError.i, rollRateError.d, rollRateError.s, rollRateError.k };

        const pid_error_t pitchRateError = _sh.PIDS[PITCH_RATE_DPS].get_error();
        telemetry.pitchRateError = { pitchRateError.p, pitchRateError.i, pitchRateError.d, pitchRateError.s, pitchRateError.k };

        const pid_error_t yawRateError = _sh.PIDS[YAW_RATE_DPS].get_error();
        telemetry.yawRateError = { yawRateError.p, yawRateError.i, yawRateError.d, yawRateError.s, yawRateError.k };

        const pid_error_t rollAngleError = _sh.PIDS[ROLL_ANGLE_DEGREES].get_error();
        telemetry.rollAngleError = { rollAngleError.p, rollAngleError.i, rollAngleError.d, rollAngleError.s, rollAngleError.k };

        const pid_error_t pitchAngleError = _sh.PIDS[PITCH_ANGLE_DEGREES].get_error();
        telemetry.pitchAngleError = { pitchAngleError.p, pitchAngleError.i, pitchAngleError.d, pitchAngleError.s, pitchAngleError.k };
    } else {
        telemetry.rollRateError =  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.pitchRateError = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.yawRateError =   { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.rollAngleError = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.pitchAngleError ={ 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
    }
    return telemetry;
}
