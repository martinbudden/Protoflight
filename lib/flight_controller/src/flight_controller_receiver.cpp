#include "dynamic_notch_filter.h"
#include "flight_controller.h"
#include <debug.h>

// NOLINTBEGIN(cppcoreguidelines-macro-usage,bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
// #defines to catch inadvertent use of _fcM or _ahM in this file.
#define _fcM "error not modifiable in this task"
#define _ahM "error not modifiable in this task"
// NOLINTEND(cppcoreguidelines-macro-usage,bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)


/*!
NOTE: CALLED FROM INITIALIZATION
*/
void FlightController::set_yaw_spin_threshold_dps(float yawSspin_threshold_dps)
{
#if defined(USE_YAW_SPIN)
    _sh.yawSspin_threshold_dps = yawSspin_threshold_dps;
#else
    (void)yawSspin_threshold_dps;
#endif
}

void FlightController::initialize_setpoint_filters(float setpoint_delta_t) // NOLINT(readability-make-member-function-const)
{
    if (_anti_gravity_config.cutoff_hz == 0) {
        _sh.anti_gravityThrottleFilter.set_to_passthrough();
    } else {
        _sh.anti_gravityThrottleFilter.set_cutoff_frequency(_anti_gravity_config.cutoff_hz, setpoint_delta_t);
    }
    // Feedforward filters
    if (_filters_config.rc_smoothing_feedforward_cutoff == 0) {
        for (auto& filter : _sh.setpoint_derivative_filters) {
            filter.set_to_passthrough();
        }
    } else {
        for (auto& filter : _sh.setpoint_derivative_filters) {
            filter.set_cutoff_frequency_and_reset(_filters_config.rc_smoothing_feedforward_cutoff, setpoint_delta_t);
        }
    }

#if defined(USE_DMAX)
    for (auto& filter : _sh.dmaxRange_filters) {
        filter.set_cutoff_frequency(DMAX_RANGE_HZ, setpoint_delta_t);
    }
    for (auto& filter : _sh.dmaxLowpass_filters) {
        filter.set_cutoff_frequency(DMAX_LOWPASS_HZ, setpoint_delta_t);
    }
#endif
#if defined(USE_ITERM_RELAX)
    for (auto& filter : _sh.iterm_relax_filters) {
        filter.set_cutoff_frequency(_iterm_relax_config.iterm_relax_cutoff, setpoint_delta_t);
    }
#endif
}

/*!
NOTE: CALLED FROM WITHIN THE RECEIVER TASK

Dynamic PID adjustments made when throttle changes:

These include:

Throttle PID Attenuation: lowers the roll rate and pitch rate P and D terms when the throttle is high

Anti-gravity: adjusts the roll rate and pitch rate P and I terms when the throttle is moving quickly

DMAX is not applied here, since it depends on the gyro value and so needs to be calculated in update_outputs_using_pids()
*/
void FlightController::apply_dynamic_pid_adjustments_on_throttle_change(float throttle, uint32_t tick_count, Debug& debug)
{
    // We don't know the period at which setpoints are updated (it depends on the receiver) so calculate this
    // so we can set the filters' cutoff frequency
    if (_rxM.setpointTickCountCounter != 0) {
        _rxM.setpointTickCountSum += tick_count;
        --_rxM.setpointTickCountCounter;
        if (_rxM.setpointTickCountCounter == 0) {
            _rxM.setpoint_delta_t = 0.001F * static_cast<float>(_rxM.setpointTickCountSum) / static_cast<float>(rx_t::SETPOINT_TICKCOUNT_COUNTER_START);
            initialize_setpoint_filters(_rxM.setpoint_delta_t);
        }
    }

    const float throttleDelta = std::fabs(throttle - _rxM.throttle_previous);
    _rxM.throttle_previous = throttle;
    const float delta_t = static_cast<float>((tick_count - _rxM.setpointTickCount_previous)) * 0.001F;
    _rxM.setpointTickCount_previous = tick_count;
    float throttleDerivative = throttleDelta/delta_t;
    debug.set(DEBUG_ANTI_GRAVITY, 0, lrintf(throttleDerivative * 100));

    const float throttleReversed = 1.0F - throttle;
    throttleDerivative *= throttleReversed * throttleReversed;
    // generally focus on the low throttle period
    if (throttle > _rxM.throttle_previous) {
        throttleDerivative *= throttleReversed * 0.5F;
        // when increasing throttle, focus even more on the low throttle range
    }
    // filtering suppresses peaks relative to troughs and prolongs the anti-gravity effects
    throttleDerivative = _sh.anti_gravityThrottleFilter.filter(throttleDerivative);
    debug.set(DEBUG_ANTI_GRAVITY, 1, lrintf(throttleDerivative * 100));

    // ****
    // use anti-gravity to adjust the ITerms on roll and pitch
    // ****

    static constexpr float ANTIGRAVITY_KI = 0.34F;
    const float iTermAccelerator =  throttleDerivative * _anti_gravity.IGain * ANTIGRAVITY_KI;
    _sh.PIDS[ROLL_RATE_DPS].set_i(_fcC.pid_constants[ROLL_RATE_DPS].ki + iTermAccelerator);
    _sh.PIDS[PITCH_RATE_DPS].set_i(_fcC.pid_constants[PITCH_RATE_DPS].ki + iTermAccelerator);
    debug.set(DEBUG_ANTI_GRAVITY, 2, lrintf(1.0F + iTermAccelerator/_sh.PIDS[PITCH_RATE_DPS].get_i()*1000.0F));


    // ****
    // calculate the Throttle PID Attenuation (TPA)
    // TPA is applied here to the PTerms on roll and pitch, and is used as a multiplier
    // of the DTERM in update_outputs_using_pids.
    // ****

    // _TPA is 1.0F (ie no attenuation) if throttle_stick <= _tpaBreakpoint;
    _rxM.TPA = 1.0F - _tpa.multiplier * std::fminf(0.0F, throttle - _tpa.breakpoint);
    debug.set(DEBUG_TPA, 0, lrintf(_rxM.TPA * 1000));

    // ****
    // use TPA and anti-gravity to adjust the PTerms on roll and pitch
    // ****

    // attenuate roll if setpoint greater than 50 DPS, half at 100 DPS
    const float attenuatorRoll = std::fmaxf(std::fabs(_sh.PIDS[ROLL_RATE_DPS].get_setpoint()) / 50.0F, 1.0F);
    const float PTermBoostRoll = 1.0F + (throttleDerivative *_anti_gravity.PGain / attenuatorRoll);
    _sh.PIDS[ROLL_RATE_DPS].set_p(_fcC.pid_constants[ROLL_RATE_DPS].kp * PTermBoostRoll * _rxM.TPA);

    // attenuate pitch if setpoint greater than 50 DPS, half at 100 DPS
    const float attenuatorPitch = std::fmaxf(std::fabs(_sh.PIDS[PITCH_RATE_DPS].get_setpoint()) / 50.0F, 1.0F);
    const float PTermBoostPitch = 1.0F + (throttleDerivative *_anti_gravity.PGain / attenuatorPitch);
    _sh.PIDS[PITCH_RATE_DPS].set_p(_fcC.pid_constants[PITCH_RATE_DPS].kp * PTermBoostPitch * _rxM.TPA);
    debug.set(DEBUG_ANTI_GRAVITY, 3, lrintf(PTermBoostPitch * 1000.0F));
}

void FlightController::clear_dynamic_pid_adjustments()
{
    _rxM.TPA = 1.0F;
    _sh.PIDS[ROLL_RATE_DPS].set_i(_fcC.pid_constants[ROLL_RATE_DPS].ki);
    _sh.PIDS[PITCH_RATE_DPS].set_i(_fcC.pid_constants[PITCH_RATE_DPS].ki);
    _sh.PIDS[ROLL_RATE_DPS].set_p(_fcC.pid_constants[ROLL_RATE_DPS].kp);
    _sh.PIDS[PITCH_RATE_DPS].set_p(_fcC.pid_constants[PITCH_RATE_DPS].kp);
}

/*!
NOTE: CALLED FROM WITHIN THE RECEIVER TASK

Sets the control mode.
*/
void FlightController::set_control_mode(fc_control_mode_e control_mode)
{
    if (control_mode == _rxM.control_mode) {
        return;
    }
    _rxM.control_mode = control_mode;
    // reset the PID integral values when we change control mode
    for (auto& pid : _sh.PIDS) {
        pid.reset_integral();
    }
}


/*!
NOTE: CALLED FROM WITHIN THE RECEIVER TASK

Use the new joystick values from the receiver to update the PID setpoints
using the NED (North-East-Down) coordinate convention.

NOTE: this function is called from `updateControls()` in the ReceiverTask,
as a result of receiving new values from the receiver.
How often it is called depends on the type of transmitter and receiver,
but is typically at intervals of between 40 milliseconds and 5 milliseconds (ie 25Hz to 200Hz).
In particular it runs much less frequently than `update_outputs_using_pids()` which typically runs at 1000Hz to 8000Hz.
*/
void FlightController::update_setpoints(const fc_controls_t& controls, failsafe_e failsafe, Debug& debug)
{
    detect_crash_or_spin();

    set_control_mode(controls.control_mode);
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    if (_dynamic_notch_filter) {
        _dynamic_notch_filter->set_throttle(controls.throttle_stick);
    }
#endif

    // output throttle may be changed by spin recovery
    _sh.output_throttle = controls.throttle_stick;

    if (failsafe == FAILSAFE_ON || _sh.crash_detected || _sh.yaw_spin_recovery || _sh.crash_flip_mode_active) {
        clear_dynamic_pid_adjustments();
    } else {
        apply_dynamic_pid_adjustments_on_throttle_change(controls.throttle_stick, controls.tick_count, debug);
    }

    //
    // Roll axis
    //
    // Pushing the ROLL stick to the right gives a positive value of roll_stick and we want this to be left side up.
    // For NED left side up is positive roll, so sign of setpoint is same sign as roll_stick.
    // So sign of _roll_stick is left unchanged.
    if (!_rxM.use_angle_mode) {
        _sh.PIDS[ROLL_RATE_DPS].set_setpoint(controls.roll_stick_dps);
    }
    if (_rxM.setpoint_delta_t != 0) {
        if (failsafe == FAILSAFE_ON || _sh.crash_detected || _sh.yaw_spin_recovery || _sh.crash_flip_mode_active) {
            _sh.PIDS[ROLL_RATE_DPS].set_setpoint_derivative(0.0F);
        } else {
            float setpoint_derivative = _sh.PIDS[ROLL_RATE_DPS].get_setpoint_delta() / _rxM.setpoint_delta_t;
            setpoint_derivative = _sh.setpoint_derivative_filters[ROLL_RATE_DPS].filter(setpoint_derivative);
            _sh.PIDS[ROLL_RATE_DPS].set_setpoint_derivative(setpoint_derivative);
        }
    }
#if defined(USE_ITERM_RELAX)
    _rxM.setpointLPs[ROLL_RATE_DPS] = _sh.iterm_relax_filters[ROLL_RATE_DPS].filter(controls.roll_stick_dps);
    _rxM.setpointHPs[ROLL_RATE_DPS] = std::fabs(controls.roll_stick_dps - _rxM.setpointLPs[ROLL_RATE_DPS]);
#endif
    _sh.PIDS[ROLL_ANGLE_DEGREES].set_setpoint(controls.roll_stick_degrees);
#if defined(USE_SIN_ANGLE_PIDS)
    _sh.PIDS[ROLL_SIN_ANGLE].set_setpoint(sinf(controls.roll_stick_degrees * DEGREES_TO_RADIANS));
#endif

    //
    // Pitch axis
    //
    // Pushing the  PITCH stick forward gives a positive value of _pitch_stick and we want this to be nose down.
    // For NED nose down is negative pitch, so sign of setpoint is opposite sign as _pitch_stick.
    // So sign of _pitch_stick is negated.
    if (!_rxM.use_angle_mode) {
        _sh.PIDS[PITCH_RATE_DPS].set_setpoint(-controls.pitch_stick_dps);
    }
    if (_rxM.setpoint_delta_t != 0) {
        if (failsafe == FAILSAFE_ON || _sh.crash_detected || _sh.yaw_spin_recovery || _sh.crash_flip_mode_active) {
            _sh.PIDS[PITCH_RATE_DPS].set_setpoint_derivative(0.0F);
        } else {
            float setpoint_derivative = _sh.PIDS[PITCH_RATE_DPS].get_setpoint_delta() / _rxM.setpoint_delta_t;
            setpoint_derivative = _sh.setpoint_derivative_filters[PITCH_RATE_DPS].filter(setpoint_derivative);
            _sh.PIDS[PITCH_RATE_DPS].set_setpoint_derivative(setpoint_derivative);
        }
    }
#if defined(USE_ITERM_RELAX)
    _rxM.setpointLPs[PITCH_RATE_DPS] = _sh.iterm_relax_filters[PITCH_RATE_DPS].filter(controls.pitch_stick_dps);
    _rxM.setpointHPs[PITCH_RATE_DPS] = std::fabs(controls.pitch_stick_dps - _rxM.setpointLPs[PITCH_RATE_DPS]);
#endif
    _sh.PIDS[PITCH_ANGLE_DEGREES].set_setpoint(-controls.pitch_stick_degrees);
#if defined(USE_SIN_ANGLE_PIDS)
    _sh.PIDS[ROLL_SIN_ANGLE].set_setpoint(sinf(-controls.pitch_stick_degrees * DEGREES_TO_RADIANS));
#endif

    //
    // Yaw axis
    //
    // Pushing the YAW stick to the right gives a positive value of _yaw_stick and we want this to be nose right.
    // For NED nose left is positive yaw, so sign of setpoint is same as sign of _yaw_stick.
    // So sign of _yaw_stick is left unchanged.
    _sh.PIDS[YAW_RATE_DPS].set_setpoint(controls.yaw_stick_dps);
    _rxM.yaw_rate_setpoint_dps = controls.yaw_stick_dps;

    //
    // Modes
    //
    // When in ground mode, the PID I-terms are set to zero to avoid integral windup on the ground
    if (_sh.ground_mode) {
        // exit ground mode if the throttle has been above _take_off_throttle_threshold for _take_off_tick_threshold ticks
        if (_sh.output_throttle < _take_off_throttle_threshold) {
            _sh.takeOffCountStart = 0;
        } else {
            const uint32_t tick_count = controls.tick_count;
            if (_sh.takeOffCountStart == 0) {
                _sh.takeOffCountStart = tick_count;
            }
            if (tick_count - _sh.takeOffCountStart > _take_off_tick_threshold) {
                _sh.ground_mode = false;
                // we've exited ground mode, so we can turn on PID integration
                switch_pid_integration_on();
            }
        }
    }
    // Angle Mode is used if the control_mode is set to angle mode, or failsafe is on.
    // Angle Mode is prevented when in Ground Mode, so the aircraft doesn't try and self-level while it is still on the ground.
    // This value is cached here, to avoid evaluating a reasonably complex condition in update_outputs_using_pids()
    _rxM.use_angle_mode = (_rxM.control_mode >= FC_CONTROL_MODE_ANGLE) && !_sh.ground_mode;
    _rxM.use_level_race_mode = (_rxM.control_mode == FC_CONTROL_MODE_LEVEL_RACE) || _flight_mode_config.level_race_mode;
}

/*!
NOTE: CALLED FROM WITHIN THE RECEIVER TASK

Detect crash or yaw spin. Runs in context of Receiver Task.
*/
void FlightController::detect_crash_or_spin()
{
#if defined(USE_YAW_SPIN_RECOVERY)
    if (_sh.yawSspin_threshold_dps !=0.0F && std::fabs(_sh.PIDS[YAW_RATE_DPS].get_previous_measurement()) > _sh.yawSspin_threshold_dps) {
        // yaw spin detected
        _sh.yaw_spin_recovery = true;
        switch_pid_integration_off();
    }
#endif
#if defined(USE_CRASH_RECOVERY)
    const size_t axis = YAW_RATE_DPS;
    const PidController pid = _sh.PIDS[axis];
    if (std::fabs(pid.get_error_raw_d()) > _crash.Dterm_threshold_dpsPS
        && std::fabs(pid.get_error_raw_p()) > _crash.gyro_threshold_dps
        && std::fabs(pid.get_setpoint()) < _crash.setpoint_threshold_dps) {
        _sh.crash_detected = true;
        switch_pid_integration_off();
    }
#endif
}
