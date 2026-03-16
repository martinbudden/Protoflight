#include "flight_controller.h"

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
    _sh.anti_gravityThrottleFilter.set_to_passthrough();

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

const std::string& FlightController::get_pid_name(pid_index_e pid_index) const
{
    return PID_NAMES[pid_index];
}

pid_error_t FlightController::get_pid_error(size_t index) const
{
    const pid_error_t error = _sh.PIDS[index].get_error();
    return pid_error_t {
        .p = error.p,
        .i = error.i,
        .d = error.d,
        .s = error.s,
        .k = error.k
    };
}

float FlightController::get_pid_setpoint(size_t index) const
{
    return _sh.PIDS[index].get_setpoint();
}

//!!TODO: reconcile FlightController::get_pid_msp, FlightController::get_pid_constants and set variants
pid_constants_uint16_t FlightController::get_pid_msp(size_t index) const
{
    assert(index < PID_COUNT);

    const auto pid_index = static_cast<pid_index_e>(index);
    const pid_constants_uint16_t ret = {
        .kp = static_cast<uint16_t>(_sh.PIDS[pid_index].get_p() / _scale_factors.kp),
        .ki = static_cast<uint16_t>(_sh.PIDS[pid_index].get_i() / _scale_factors.ki),
        .kd = static_cast<uint16_t>(_sh.PIDS[pid_index].get_d() / _scale_factors.kd),
        .ks = static_cast<uint16_t>(_sh.PIDS[pid_index].get_s() / _scale_factors.ks),
        .kk = static_cast<uint16_t>(_sh.PIDS[pid_index].get_k() / _scale_factors.kk),
    };
    return ret;
}

pid_constants_uint16_t FlightController::get_pid_constants(pid_index_e pid_index) const
{
    const pid_constants_t pid = _sh.PIDS[pid_index].get_pid();
    const pid_constants_uint16_t pid16 = {
        static_cast<uint16_t>(std::lroundf(pid.kp / _scale_factors.kp)),
        static_cast<uint16_t>(std::lroundf(pid.ki / _scale_factors.ki)),
        static_cast<uint16_t>(std::lroundf(pid.kd / _scale_factors.kd)),
        static_cast<uint16_t>(std::lroundf(pid.ks / _scale_factors.ks)),
        static_cast<uint16_t>(std::lroundf(pid.kk / _scale_factors.kk))
    };

    return pid16;
}

/*!
Set the P, I, D, S, and K values for the PID with index pid_index.
Integration is switched off, so that there is no integral windup before takeoff.
*/
void FlightController::set_pid_constants(pid_index_e pid_index, const pid_constants_uint16_t& pid16)
{

    const pid_constants_t pid = {
        static_cast<float>(pid16.kp) * _scale_factors.kp,
        static_cast<float>(pid16.ki) * _scale_factors.ki,
        static_cast<float>(pid16.kd) * _scale_factors.kd,
        static_cast<float>(pid16.ks) * _scale_factors.ks,
        static_cast<float>(pid16.kk) * _scale_factors.kk
    };

    _sh.PIDS[pid_index].set_pid(pid);
    _sh.PIDS[pid_index].switch_integration_off();
    _sh.PIDS[pid_index].set_setpoint(0.0F);
    // keep copies of the PID constants so they can be adjusted by anti-gravity
    _fcM.pid_constants[pid_index] = _sh.PIDS[pid_index].get_pid();
}

void FlightController::set_pid_p_msp(pid_index_e pid_index, uint16_t kp)
{
    _sh.PIDS[pid_index].set_p(kp * _scale_factors.kp);
    _fcM.pid_constants[pid_index].kp = _sh.PIDS[pid_index].get_p();
}

void FlightController::set_pid_pd_msp(pid_index_e pid_index, uint16_t kp)
{
    const PidController pid = get_pid(pid_index);
    const float ratio = pid.get_d() / pid.get_p();
    set_pid_p_msp(pid_index, kp);
    set_pid_d_msp(pid_index, static_cast<uint16_t>(static_cast<float>(kp)*ratio));
}

void FlightController::set_pid_i_msp(pid_index_e pid_index, uint16_t ki)
{
    _sh.PIDS[pid_index].set_i(ki * _scale_factors.ki);
    _fcM.pid_constants[pid_index].ki = _sh.PIDS[pid_index].get_i();
}

void FlightController::set_pid_d_msp(pid_index_e pid_index, uint16_t kd)
{
    _sh.PIDS[pid_index].set_d(kd * _scale_factors.kd);
    _fcM.pid_constants[pid_index].kd = _sh.PIDS[pid_index].get_d();
}

void FlightController::set_pid_s_msp(pid_index_e pid_index, uint16_t ks)
{
    _sh.PIDS[pid_index].set_s(ks * _scale_factors.ks);
    _fcM.pid_constants[pid_index].ks = _sh.PIDS[pid_index].get_s();
}

void FlightController::set_pid_k_msp(pid_index_e pid_index, uint16_t kk)
{
    _sh.PIDS[pid_index].set_k(kk * _scale_factors.kk);
    _fcM.pid_constants[pid_index].kk = _sh.PIDS[pid_index].get_k();
}

const simplified_pid_settings_t& FlightController::get_simplified_pid_Settings() const
{
    return _fcM.simplified_pid_settings;
}

void FlightController::set_simplified_pid_Settings(const simplified_pid_settings_t& settings)
{
    _fcM.simplified_pid_settings = settings;
    const float master_multiplier = static_cast<float>(settings.multiplier) / 100.0F;
    const float piGain = static_cast<float>(settings.pi_gain) / 100.0F;
    const float dGain = static_cast<float>(settings.d_gain) / 100.0F;
    const float kGain = static_cast<float>(settings.k_gain) / 100.0F;
    const float iGain = static_cast<float>(settings.i_gain) / 100.0F;

    pid_constants_uint16_t pid16 {};

    pid16.kp = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[ROLL_RATE_DPS].kp * master_multiplier * piGain), uint16_t{0}, PID_GAIN_MAX);
    pid16.ki = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[ROLL_RATE_DPS].ki * master_multiplier * piGain * iGain), uint16_t{0}, PID_GAIN_MAX);
    pid16.kd = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[ROLL_RATE_DPS].kd * master_multiplier * dGain), uint16_t{0}, PID_GAIN_MAX);
    pid16.kk = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[ROLL_RATE_DPS].kk * master_multiplier * kGain), uint16_t{0}, K_GAIN_MAX);
    set_pid_constants(ROLL_RATE_DPS, pid16);

    pid16.kp = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[PITCH_RATE_DPS].kp * master_multiplier * piGain), uint16_t{0}, PID_GAIN_MAX);
    const float pitchPI_gain = static_cast<float>(settings.pitch_pi_gain) / 100.0F;
    pid16.ki = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[PITCH_RATE_DPS].ki * master_multiplier * piGain * iGain * pitchPI_gain), uint16_t{0}, PID_GAIN_MAX);
    const float pitchDGain = static_cast<float>(settings.roll_pitch_ratio) / 100.0F;
    pid16.kd = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[PITCH_RATE_DPS].kd * master_multiplier * dGain * pitchDGain), uint16_t{0}, PID_GAIN_MAX);
    pid16.kk = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[PITCH_RATE_DPS].kk * master_multiplier * kGain), uint16_t{0}, K_GAIN_MAX);
    set_pid_constants(PITCH_RATE_DPS, pid16);

    if (_pid_tuning_mode == PID_TUNING_SIMPLIFIED_RPY) {
        pid16.kp = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[YAW_RATE_DPS].kp * master_multiplier * piGain), uint16_t{0}, PID_GAIN_MAX);
        pid16.ki = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[YAW_RATE_DPS].ki * master_multiplier * piGain * iGain), uint16_t{0}, PID_GAIN_MAX);
        pid16.kd = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[YAW_RATE_DPS].kd * master_multiplier * dGain), uint16_t{0}, PID_GAIN_MAX);
        pid16.kk = std::clamp(static_cast<uint16_t>(DEFAULT_PIDS[YAW_RATE_DPS].kk * master_multiplier * kGain), uint16_t{0}, K_GAIN_MAX);
        set_pid_constants(YAW_RATE_DPS, pid16);
    }

#if defined(USE_DMAX)
    const float dmaxGainRoll = static_cast<float>(settings.dmax_gain + (100 - settings.dmax_gain) * DEFAULT_PIDS[ROLL_RATE_DPS].kd) / (100.0F*DMAX_DEFAULT_ROLL);
    _dmax_config.dmax[ROLL_RATE_DPS] = std::clamp(static_cast<uint8_t>(DMAX_DEFAULT_ROLL * master_multiplier * dGain * dmaxGainRoll), uint8_t{0}, uint8_t{PID_GAIN_MAX});

    const float dmaxGainPitch = static_cast<float>(settings.dmax_gain + (100 - settings.dmax_gain) * DEFAULT_PIDS[PITCH_RATE_DPS].kd) / (100.0F*DMAX_DEFAULT_PITCH);
    _dmax_config.dmax[PITCH_RATE_DPS] = std::clamp(static_cast<uint8_t>(DMAX_DEFAULT_PITCH * master_multiplier * dGain * pitchDGain * dmaxGainPitch), uint8_t{0}, uint8_t{PID_GAIN_MAX});
#endif
}

uint32_t FlightController::get_output_power_time_microseconds() const
{
    //return _motor_mixer.get_output_power_time_microseconds();
    return 0;
}

void FlightController::motors_switch_off(MotorMixerBase& motor_mixer)
{
    motor_mixer.motors_switch_off();
    _sh.takeOffCountStart = 0;
    _sh.ground_mode = true;
    switch_pid_integration_off();
}

void FlightController::motors_switch_on(MotorMixerBase& motor_mixer)
{
    // don't allow motors to be switched on if the sensor fusion has not initialized
#if defined(FRAMEWORK_TEST)
    if (!_sensor_fusion_filter_is_initializing) { //!!TODO: fix _sensorFusionFilterIsInitializing
#else
    {
#endif
        motor_mixer.motors_switch_on();
        // reset the PID integral values when we switch the motors on
        switch_pid_integration_on();
    }
}

void FlightController::set_blackbox_active(bool is_active)
{
    _sh.blackbox_active = is_active;
}

void FlightController::set_pid_tuning_mode(pid_tuning_mode_e pid_tuning_mode)
{
    _pid_tuning_mode = pid_tuning_mode;
}

void FlightController::set_max_angle_rates(float max_roll_rate_dps, float max_pitch_rate_dps, float maxYaw_rate_dps)
{
    _max_roll_rate_dps = max_roll_rate_dps;
    _max_pitch_rate_dps = max_pitch_rate_dps;
    _maxYaw_rate_dps = maxYaw_rate_dps;
}

void FlightController::set_filters_config(const flight_controller_filters_config_t& filters_config)
{
    _filters_config = filters_config;

    // always used PT1 filters for DTerm filters.
#if defined(USE_DTERM_FILTERS_EXTENDED)
    _filters_config.dterm_lpf1_type = flight_controller_filters_config_t::PT1;
    _filters_config.dterm_lpf2_type = flight_controller_filters_config_t::PT1;
#endif

    //!!TODO: check dT value for filters config
    const float delta_t = static_cast<float>(_task_interval_microseconds) * 0.000001F;
    //const float delta_t = (static_cast<float>(_ahrs.get_task_interval_microseconds()) * 0.000001F) / static_cast<float>(_outputToMotorsDenominator);

    // DTerm filters
    if (_filters_config.dterm_lpf1_hz == 0) {
        for (auto& filter : _sh.dterm_filters1) {
            filter.set_to_passthrough();
        }
    } else {
        // DTerm filters always use a PowerTransfer1 filter.
        for (auto& filter : _sh.dterm_filters1) {
            filter.set_cutoff_frequency_and_reset(filters_config.dterm_lpf1_hz, delta_t);
        }
    }
    if (_filters_config.dterm_lpf2_hz == 0) {
        for (auto& filter : _sh.dterm_filters2) {
            filter.set_to_passthrough();
        }
    } else {
        // DTerm filters always use a PowerTransfer1 filter.
        for (auto& filter : _sh.dterm_filters2) {
            filter.set_cutoff_frequency_and_reset(filters_config.dterm_lpf2_hz, delta_t);
        }
    }

    // Output filters
    if (_filters_config.output_lpf_hz == 0) {
        for (auto& filter : _sh.output_filters) {
            filter.set_to_passthrough();
        }
    } else {
        for (auto& filter : _sh.output_filters) {
            filter.set_cutoff_frequency_and_reset(filters_config.output_lpf_hz, delta_t);
        }
    }
}

void FlightController::set_flight_mode_config(const flight_mode_config_t& flight_mode_config)
{
    _flight_mode_config = flight_mode_config;
}

void FlightController::set_tpa_config(const tpa_config_t& tpa_config)
{
    _tpa_config = tpa_config;

    // default of 1350 gives 0.35. range is limited to 0 to 0.99
    enum { PWM_RANGE_MIN = 1000 };
    _tpa.breakpoint = std::clamp(static_cast<float>(tpa_config.tpa_breakpoint - PWM_RANGE_MIN) / 1000.0F, 0.0F, 0.99F);
    _tpa.multiplier = (static_cast<float>(tpa_config.tpa_rate) / 100.0F) / (1.0F - _tpa.breakpoint);

    // ensure tpaLowBreakpoint is always <= tpaBreakpoint
    _tpa.lowBreakpoint = std::fminf(std::clamp(static_cast<float>(tpa_config.tpa_low_breakpoint - PWM_RANGE_MIN) / 1000.0F, 0.01F, 1.0F), _tpa.breakpoint);
}

void FlightController::set_anti_gravity_config(const anti_gravity_config_t& anti_gravity_config)
{
    _anti_gravity_config = anti_gravity_config;
    _anti_gravity = {
        .PGain = static_cast<float>(anti_gravity_config.p_gain) * _scale_factors.kp,
        .IGain = static_cast<float>(anti_gravity_config.i_gain) * _scale_factors.ki
    };
}

void FlightController::set_crash_flip_config(const crash_flip_config_t& crash_flip_config)
{
    _crash_flip_config = crash_flip_config;
}

#if defined(USE_DMAX)
void FlightController::set_dmax_config(const dmax_config_t& dmax_config) // NOLINT(readability-make-member-function-const)
{
    _dmax_config = dmax_config;
#if (__cplusplus >= 202002L)
    for (auto axis : std::views::iota(size_t{0}, size_t{RPY_AXIS_COUNT})) {
#else
    for (size_t axis = 0; axis < RPY_AXIS_COUNT; ++axis) {
#endif
        const uint8_t dmax = dmax_config.dmax[axis];
        const pid_constants_uint16_t pid16 = get_pid_constants(static_cast<pid_index_e>(axis));
        if (pid16.kd > 0 && dmax > pid16.kd) {
            // ratio of DMax to kd, eg if kd is 8 and DMax is 10 then dmaxPercent is 1.25
            _dmax.percent[axis] = static_cast<float>(dmax) / pid16.kd;
        } else {
            _dmax.percent[axis] = 1.0F;
        }
    }
    _dmax.gyroGain = DMAX_GAIN_FACTOR * static_cast<float>(dmax_config.dmax_gain) / DMAX_LOWPASS_HZ;
    // lowpass included inversely in gain since stronger lowpass decreases peak effect
    _dmax.setpointGain = DMAX_SETPOINT_GAIN_FACTOR * static_cast<float>(dmax_config.dmax_gain * dmax_config.dmax_advance) / 100.0F / DMAX_LOWPASS_HZ;
    for (auto& filter : _sh.dmaxRange_filters) {
        filter.set_to_passthrough();
    }
    for (auto& filter : _sh.dmaxLowpass_filters) {
        filter.set_to_passthrough();
    }
}
#endif

#if defined(USE_ITERM_RELAX)
void FlightController::set_iterm_relax_config(const iterm_relax_config_t& iterm_relax_config)
{
    _iterm_relax_config = iterm_relax_config;
    _iterm_relax.setpoint_threshold_dps = iterm_relax_config.iterm_relax_setpoint_threshold;
    for (auto& filter : _sh.iterm_relax_filters) {
        filter.set_to_passthrough();
    }
}
#endif

#if defined(USE_YAW_SPIN_RECOVERY)
void FlightController::set_yaw_spin_recovery_config(const yaw_spin_recovery_config_t& yaw_spin_recovery_config)
{
    _yaw_spin_recovery_config = yaw_spin_recovery_config;
}
#endif

#if defined(USE_CRASH_RECOVERY)
void FlightController::set_crash_recovery_config(const crash_recovery_config_t& crash_recovery_config)
{
    _crash_recovery_config = crash_recovery_config;
    _crash = {
        .timeLimitUs = static_cast<uint32_t>(crash_recovery_config.crash_time * 1000),
        .timeDelayUs = static_cast<uint32_t>(crash_recovery_config.crash_delay * 1000),
        .recoveryAngleDeciDegrees = crash_recovery_config.crash_recovery_angle * 10,
        .recoveryRate = static_cast<float>(crash_recovery_config.crash_recovery_rate),
        .gyro_threshold_dps = static_cast<float>(crash_recovery_config.crash_gthreshold), // error in deg/s
        .Dterm_threshold_dpsPS = static_cast<float>(crash_recovery_config.crash_dthreshold * 1000), // gyro delta in deg/s/s * 1000 to match original 2017 intent
        .setpoint_threshold_dps = static_cast<float>(crash_recovery_config.crash_setpoint_threshold),
        .limitYaw = static_cast<float>(crash_recovery_config.crash_limit_yaw)
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
flight_controller_quadcopter_telemetry_t FlightController::get_telemetry_data(const MotorMixerBase& motor_mixer) const
{
    flight_controller_quadcopter_telemetry_t telemetry;

#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{motor_mixer.get_motor_count()})) {
#else
    for (size_t ii = 0; ii < motor_mixer.get_motor_count(); ++ ii) {
#endif
        telemetry.motors[ii].power = motor_mixer.get_motor_output(ii);
        telemetry.motors[ii].rpm = motor_mixer.get_motor_rpm(ii);
    }
    if (motor_mixer.motors_is_on()) {
        const pid_error_t roll_rate_error = _sh.PIDS[ROLL_RATE_DPS].get_error();
        telemetry.roll_rate_error = { roll_rate_error.p, roll_rate_error.i, roll_rate_error.d, roll_rate_error.s, roll_rate_error.k };

        const pid_error_t pitch_rate_error = _sh.PIDS[PITCH_RATE_DPS].get_error();
        telemetry.pitch_rate_error = { pitch_rate_error.p, pitch_rate_error.i, pitch_rate_error.d, pitch_rate_error.s, pitch_rate_error.k };

        const pid_error_t yaw_rate_error = _sh.PIDS[YAW_RATE_DPS].get_error();
        telemetry.yaw_rate_error = { yaw_rate_error.p, yaw_rate_error.i, yaw_rate_error.d, yaw_rate_error.s, yaw_rate_error.k };

        const pid_error_t roll_angle_error = _sh.PIDS[ROLL_ANGLE_DEGREES].get_error();
        telemetry.roll_angle_error = { roll_angle_error.p, roll_angle_error.i, roll_angle_error.d, roll_angle_error.s, roll_angle_error.k };

        const pid_error_t pitch_angle_error = _sh.PIDS[PITCH_ANGLE_DEGREES].get_error();
        telemetry.pitch_angle_error = { pitch_angle_error.p, pitch_angle_error.i, pitch_angle_error.d, pitch_angle_error.s, pitch_angle_error.k };
    } else {
        telemetry.roll_rate_error =  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.pitch_rate_error = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.yaw_rate_error =   { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.roll_angle_error = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
        telemetry.pitch_angle_error ={ 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };
    }
    return telemetry;
}
