#include "flight_controller.h"

#include <ahrs.h>
#include <ahrs_message_queue.h>
#include <debug.h>
#include <motor_mixer_message_queue.h>
#include <time_microseconds.h>

#if (__cplusplus >= 202002L)
#include <ranges>
#endif


// NOLINTBEGIN(cppcoreguidelines-macro-usage,bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
// #defines to catch inadvertent use of _fcM or _rxM in this file.
#define _fcM "error not modifiable in this task"
#define _rxM "error not modifiable in this task"
// NOLINTEND(cppcoreguidelines-macro-usage,bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK

Calculate the dmax_multipliers.

This are multipliers that are applied to the roll and pitch axis DTerms.
This means DTerms can be low in normal flight but are boosted to a higher value when required.
They are boosted when the DTerm error is small and the setpoint change is also small.
*/
void FlightController::calculate_dmax_multipliers(Debug& debug)
{
#if defined(USE_DMAX)
#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{RP_AXIS_COUNT})) {
#else
    for (size_t ii = 0; ii <= RP_AXIS_COUNT; ++ii) {
#endif
        _ahM.dmax_multiplier[ii] = 1.0F;
        if (_dmax.percent[ii] > 1.0F) {
            const float delta_t = static_cast<float>(_task_interval_microseconds) * 0.000001F;
            const float gyroDeltaD = delta_t * _sh.PIDS[ii].get_error_d(); //!!TODO: check using PID error in DMAX, surely this is too easy
            const float gyroFactor = std::fabs(_sh.dmaxRange_filters[ii].filter(gyroDeltaD)) * _dmax.gyroGain;
            const float setpointFactor = std::fabs(_sh.PIDS[ii].get_setpoint_delta()) * _dmax.setpointGain;
            const float boost = std::fmaxf(gyroFactor, setpointFactor);
            // boost starts at zero, and by 1.0 we get Dmax, but it can exceed 1.0
            _ahM.dmax_multiplier[ii] += (_dmax.percent[ii] - 1.0F) * boost;
            _ahM.dmax_multiplier[ii] = _sh.dmaxLowpass_filters[ii].filter(_ahM.dmax_multiplier[ii]);
            // limit the multiplier to _dmax.percent
            _ahM.dmax_multiplier[ii] = std::fminf(_ahM.dmax_multiplier[ii], _dmax.percent[ii]);
            if (debug.get_mode() == DEBUG_D_MAX) {
                if (ii == FD_ROLL) {
                    debug.set(DEBUG_D_MAX, 0, lrintf(gyroFactor * 100.0F));
                    debug.set(DEBUG_D_MAX, 1, lrintf(setpointFactor * 100.0F));
                    debug.set(DEBUG_D_MAX, 2, lrintf(_fcC.pid_constants[ROLL_RATE_DPS].kd * _ahM.dmax_multiplier[ROLL_RATE_DPS] * 10));
                } else {
                    debug.set(DEBUG_D_MAX, 3, lrintf(_fcC.pid_constants[PITCH_RATE_DPS].kd * _ahM.dmax_multiplier[PITCH_RATE_DPS] * 10));
                }
            }
        }
    }
#else
    (void)debug;
#endif
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK
*/
void FlightController::apply_crash_flip_to_motors(const xyz_t& gyro_rps, float delta_t, MotorMixerMessageQueue& motor_mixer_message_queue)
{
    (void)gyro_rps;
    (void)delta_t;
    (void)motor_mixer_message_queue;
}


/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK
*/
void FlightController::recover_from_yaw_spin(const xyz_t& gyro_rps, float delta_t, MotorMixerMessageQueue& motor_mixer_message_queue)
{
#if defined(USE_YAW_SPIN_RECOVERY)
    if (std::fabs(gyro_rps.z) > _yaw_spin.recovered_rps) {
        _sh.output_throttle = 0.5F; // half throttle gives maximum yaw authority, since outputs will have maximum range before being clipped
        // use the YAW_RATE_DPS PID to bring the spin down to zero
        _sh.PIDS[YAW_RATE_DPS].set_setpoint(0.0F);
        const float yaw_rate_dps = yaw_rate_ned_dps(gyro_rps);
        motor_mixer_message_queue_item_t outputs {
            .throttle = _sh.output_throttle,
            .roll_dps = 0.0F,
            .pitch_dps = 0.0F,
            .yaw_dps = _sh.PIDS[YAW_RATE_DPS].update(yaw_rate_dps, delta_t)
        };
        if (std::fabs(gyro_rps.z) <= _yaw_spin.partially_recovered_rps) {
            // we have partially recovered from the spin, so try and also correct any roll and pitch spin
            _sh.PIDS[ROLL_RATE_DPS].set_setpoint(0.0F);
            const float roll_rate_dps = roll_rate_ned_dps(gyro_rps);
            outputs.roll_dps = _sh.PIDS[ROLL_RATE_DPS].update(roll_rate_dps, delta_t);

            _sh.PIDS[PITCH_RATE_DPS].set_setpoint(0.0F);
            const float pitch_rate_dps = pitch_rate_ned_dps(gyro_rps);
            outputs.pitch_dps = _sh.PIDS[PITCH_RATE_DPS].update(pitch_rate_dps, delta_t);
        }
        motor_mixer_message_queue.SIGNAL(outputs);
    } else {
        // come out of yaw spin recovery
        _sh.yaw_spin_recovery = false;
        // switch PID integration back on
        switch_pid_integration_on();
    }
#else
    (void)gyro_rps;
    (void)delta_t;
    (void)motor_mixer_message_queue;
#endif
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK

In angle mode, the roll and pitch angles are used to set the setpoints for the rollRate and pitchRate PIDs.
Level Race Mode (aka NFE(Not Fast Enough) mode) is equivalent to angle mode on roll and acro mode on pitch.
*/
void FlightController::update_rate_setpoints_for_angle_mode(const Quaternion& orientation, float delta_t) // NOLINT(readability-make-member-function-const)
{
    // convert orientationENU from the ENU coordinate frame to the NED coordinate frame
    //static const Quaternion qENUtoNED(0.0F, sqrtf(0.5F), sqrtf(0.5F), 0.0F);
    //const Quaternion orientationNED = qENUtoNED * orientationENU;
    //_ahM.roll_angle_degreesRaw = orientationNED.calculateRollDegrees();
    //_ahM.pitch_angle_degreesRaw = orientationNED.calculatePitchDegrees();

    // use the outputs from the "ANGLE" PIDS as the setpoints for the "RATE" PIDs.
    //!!TODO: need to mix in YAW to roll and pitch changes to coordinate turn

    //const float yaw_rate_setpoint_dps = _sh.PIDS[YAW_RATE_DPS].get_setpoint();

    // Running the angle PIDs in "quaternion space" rather than "angle space",
    // avoids the computationally expensive Quaternion::calculateRoll and Quaternion::calculatePitch

    //!!TODO: look at using vector product here
    if (_ahM.amcs.state == STATE_CALCULATE_ROLL) {
        if (!_rxC.use_level_race_mode) {
            // in level race mode we use angle mode on roll, acro mode on pitch
            // so we only advance calculation to pitch if not in level race mode
            _ahM.amcs.state = STATE_CALCULATE_PITCH;
        }

        _ahM.amcs.roll_sin_angle = roll_sin_angle_ned(orientation);
        float rollRateSetpointDPS;
#if defined(USE_SIN_ANGLE_PIDS)
        if (_use_quaternion_space_for_angle_mode) {
            const float roll_sin_angleDelta = _sh.dterm_filters1[ROLL_SIN_ANGLE].filter(_ahM.amcs.roll_sin_angle - _sh.PIDS[ROLL_SIN_ANGLE].get_previous_measurement());
            rollRateSetpointDPS = _sh.PIDS[ROLL_SIN_ANGLE].update_delta(_ahM.amcs.roll_sin_angle, roll_sin_angleDelta, delta_t);
        } else
#endif
        {
            const float roll_angle_degrees = roll_angle_degrees_ned(orientation);
            const float rollAngleDelta = _sh.dterm_filters1[ROLL_ANGLE_DEGREES].filter(roll_angle_degrees - _sh.PIDS[ROLL_ANGLE_DEGREES].get_previous_measurement());
            // calculate roll rate setpoint in degrees, range is [-_max_roll_angle_degrees, _max_roll_angle_degrees], typically [-60, 60]
            const float rollRateSetpointDegrees = _sh.PIDS[ROLL_ANGLE_DEGREES].update_delta(roll_angle_degrees, rollAngleDelta, delta_t);
            // convert to value in range [-1.0, 1.0] to be used for the ROLL_RATE_DPS setpoint
            rollRateSetpointDPS = std::clamp(rollRateSetpointDegrees / _max_roll_angle_degrees, -1.0F, 1.0F) * _max_roll_rate_dps;
        }
        // a component of YAW changes roll, so update accordingly !!TODO:check sign
        //rollRateSetpointDPS -= yaw_rate_setpoint_dps * _ahM.amcs.roll_sin_angle;
        _sh.PIDS[ROLL_RATE_DPS].set_setpoint(rollRateSetpointDPS);
    } else {
        _ahM.amcs.state = STATE_CALCULATE_ROLL;

        _ahM.amcs.pitch_sin_angle = pitch_sin_angle_ned(orientation);
        float pitchRateSetpointDPS;
#if defined(USE_SIN_ANGLE_PIDS)
        if (_use_quaternion_space_for_angle_mode) {
            const float pitch_sin_angleDelta = _sh.dterm_filters1[PITCH_SIN_ANGLE].filter(_ahM.amcs.pitch_sin_angle - _sh.PIDS[PITCH_SIN_ANGLE].get_previous_measurement());
            pitchRateSetpointDPS = _sh.PIDS[PITCH_SIN_ANGLE].update_delta(_ahM.amcs.pitch_sin_angle, pitch_sin_angleDelta, delta_t);
        } else
#endif
        {
            const float pitch_angle_degrees = pitch_angle_degrees_ned(orientation);
            const float pitchAngleDelta = _sh.dterm_filters1[PITCH_ANGLE_DEGREES].filter(pitch_angle_degrees - _sh.PIDS[PITCH_ANGLE_DEGREES].get_previous_measurement());
            // calculate pitch rate setpoint in degrees, range is [-_max_pitch_angle_degrees, _max_pitch_angle_degrees], typically [-60, 60]
            const float pitchRateSetpointDegrees = _sh.PIDS[PITCH_ANGLE_DEGREES].update_delta(pitch_angle_degrees, pitchAngleDelta, delta_t);
            // convert to value in range [-1.0, 1.0] to be used for the PITCH_RATE_DPS setpoint
            pitchRateSetpointDPS = std::clamp(pitchRateSetpointDegrees / _max_pitch_angle_degrees, -1.0F, 1.0F) * _max_pitch_rate_dps;
        }
        // a component of YAW changes pitch, so update accordingly !!TODO:check sign
        //pitchRateSetpoint += yaw_rate_setpoint_dps * _ahM.amcs.pitch_sin_angle;
        _sh.PIDS[PITCH_RATE_DPS].set_setpoint(pitchRateSetpointDPS);
    }

    // the cosRoll and cosPitch functions are reasonably cheap, they both involve taking a square root
    // both are positive in ANGLE mode, since absolute values of both roll and pitch angles are less than 90 degrees
#if false
    const float roll_cosAngle = roll_cos_angle_ned(orientation);
    const float pitch_cosAngle = pitch_cos_angle_ned(orientation);
    const float yaw_rate_setpoint_attenuation = fmaxf(roll_cosAngle, pitch_cosAngle);
#else
    const float roll_sin_angle_squared = _ahM.amcs.roll_sin_angle*_ahM.amcs.roll_sin_angle;
    const float pitch_sin_angle_squared = _ahM.amcs.pitch_sin_angle*_ahM.amcs.pitch_sin_angle;
    const float min_sin_angle_squared = fminf(roll_sin_angle_squared, pitch_sin_angle_squared);
    const float yaw_rate_setpoint_attenuation = sqrtf(1.0F - min_sin_angle_squared); // this is equal to fmaxf(roll_cosAngle, pitch_cosAngle)
#endif
    // attenuate yaw rate setpoint
    _sh.PIDS[YAW_RATE_DPS].set_setpoint(_rxC.yaw_rate_setpoint_dps*yaw_rate_setpoint_attenuation);
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK
*/
float FlightController::calculate_iterm_error(size_t axis, float measurement, Debug& debug)
{
    const float setpoint = _sh.PIDS[axis].get_setpoint();
    // iterm_error is just `setpoint - measurement`, if there is no ITerm relax
    float iterm_error = setpoint - measurement;
#if defined(USE_ITERM_RELAX)
    if (_iterm_relax_config.iterm_relax == ITERM_RELAX_ON) {
        const float setpointLp = _rxC.setpointLPs[axis];
        const float setpointHp = std::fabs(setpoint - setpointLp);
        float setpoint_threshold_dps = _iterm_relax.setpoint_threshold_dps;
        if (_rxC.use_angle_mode) {
            setpoint_threshold_dps *= 0.2F;
        }
        const float iterm_relax_factor = std::fmaxf(0.0F, 1.0F - setpointHp/setpoint_threshold_dps);
        iterm_error *= iterm_relax_factor;

        if (axis == ROLL_RATE_DPS && debug.get_mode() == DEBUG_ITERM_RELAX) {
            debug.set(DEBUG_ITERM_RELAX, 0, lrintf(setpointHp));
            debug.set(DEBUG_ITERM_RELAX, 1, lrintf(iterm_relax_factor));
            debug.set(DEBUG_ITERM_RELAX, 2, lrintf(iterm_error));
        }
    }
#else
    (void)debug;
#endif
    return iterm_error;
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK
It is typically called at frequency of between 1000Hz and 8000Hz, so it has to be FAST.

The FlightController uses the NED (North-East-Down) coordinate convention.
gyro_rps, acc, and orientation come from the AHRS and use the ENU (East-North-Up) coordinate convention.
*/
void FlightController::update_outputs_using_pids(const ahrs_data_t& ahrs_data, AhrsMessageQueue& ahrs_message_queue, MotorMixerMessageQueue& motor_mixer_message_queue, Debug& debug)
{
#if defined(USE_BLACKBOX) || defined(USE_BACKCHANNEL) || defined(USE_DASHBOARD)
    // signalling/sending to the message queue is not free, so, in this time-critical function, we only do it if necessary
    ++_ahM.send_blackbox_message_count;
    if (_ahM.send_blackbox_message_count >= _send_blackbox_message_denominator) {
        _ahM.send_blackbox_message_count = 0;
        if (_sh.blackbox_active) {
            ahrs_message_queue.SIGNAL(ahrs_data);
        }
#if defined(USE_BACKCHANNEL) || defined(USE_DASHBOARD)
        ahrs_message_queue.SEND_AHRS_DATA(ahrs_data);
#endif
    }
#else
    (void)ahrs_message_queue;
#endif
    if (_sh.crash_flip_mode_active) {
        apply_crash_flip_to_motors(ahrs_data.acc_gyro_rps.gyro_rps, ahrs_data.delta_t, motor_mixer_message_queue);
        return;
    }

#if defined(USE_YAW_SPIN_RECOVERY)
    if (_sh.yaw_spin_recovery) {
        recover_from_yaw_spin(ahrs_data.acc_gyro_rps.gyro_rps, ahrs_data.delta_t, motor_mixer_message_queue);
        return;
    }
#endif
#if defined(USE_DMAX)
    calculate_dmax_multipliers(debug);
#endif

#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    const time_us32_t time0 = time_us();
#endif
#if defined(USE_ANGLE_MODE_LOCKED_ON)
    update_rate_setpoints_for_angle_mode(ahrs_data.orientation, ahrs_data.delta_t);
#else
    if (_rxC.use_angle_mode) {
        update_rate_setpoints_for_angle_mode(ahrs_data.orientation, ahrs_data.delta_t);
    }
#endif
#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    const time_us32_t time1 = time_us();
    _sh.time_checks_microseconds[0] = time1 - time0;
#endif

    motor_mixer_message_queue_item_t outputs {}; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    outputs.throttle = _sh.output_throttle;

    // Use the PIDs to calculate the outputs for each axis.
    // Note that the delta-values (ie the DTerms) are filtered:
    // this is because they are especially noisy, being the derivative of a noisy value.

    // The output from the PIDs is filtered.
    // This smooths the output, but also accumulates the output in the filter,
    // so the values influence the output even when `outputToMotors` is not called.

    //
    // Roll axis
    //
//Serial.printf("RR\r\n");
    const float roll_rate_dps = roll_rate_ned_dps(ahrs_data.acc_gyro_rps.gyro_rps);
    // filter the DTerm twice
    float roll_rate_delta_filtered_dps = _sh.dterm_filters1[ROLL_RATE_DPS].filter(roll_rate_dps - _sh.PIDS[ROLL_RATE_DPS].get_previous_measurement());
    roll_rate_delta_filtered_dps = _sh.dterm_filters2[ROLL_RATE_DPS].filter(roll_rate_delta_filtered_dps);
    outputs.roll_dps = _sh.PIDS[ROLL_RATE_DPS].update_delta_iterm(
                                                    roll_rate_dps,
                                                    roll_rate_delta_filtered_dps * _rxC.TPA * _ahM.dmax_multiplier[ROLL_RATE_DPS],
                                                    calculate_iterm_error(ROLL_RATE_DPS, roll_rate_dps, debug),
                                                    ahrs_data.delta_t);
    outputs.roll_dps = _sh.output_filters[FD_ROLL].filter(outputs.roll_dps);
#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    const time_us32_t time2 = time_us();
    _sh.time_checks_microseconds[1] = time2 - time1;
#endif

    //
    // Pitch axis
    //
    const float pitch_rate_dps = pitch_rate_ned_dps(ahrs_data.acc_gyro_rps.gyro_rps);
    // filter the DTerm twice
    float pitch_rate_delta_filtered_dps = _sh.dterm_filters1[PITCH_RATE_DPS].filter(pitch_rate_dps - _sh.PIDS[PITCH_RATE_DPS].get_previous_measurement());
    pitch_rate_delta_filtered_dps = _sh.dterm_filters2[PITCH_RATE_DPS].filter(pitch_rate_delta_filtered_dps);
    outputs.pitch_dps = _sh.PIDS[PITCH_RATE_DPS].update_delta_iterm(
                                                    pitch_rate_dps,
                                                    pitch_rate_delta_filtered_dps * _rxC.TPA * _ahM.dmax_multiplier[PITCH_RATE_DPS],
                                                    calculate_iterm_error(PITCH_RATE_DPS, pitch_rate_dps, debug),
                                                    ahrs_data.delta_t);
    outputs.pitch_dps = _sh.output_filters[FD_PITCH].filter(outputs.pitch_dps);
#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    const time_us32_t time3 = time_us();
    _sh.time_checks_microseconds[2] = time3 - time2;
#endif


    //
    // Yaw axis
    //
    // DTerm is zero for yaw_rate, so call updateSPI() with no DTerm filtering, no TPA, no DMax, no ITerm relax, and no KTerm
    const float yaw_rate_dps = yaw_rate_ned_dps(ahrs_data.acc_gyro_rps.gyro_rps);
    outputs.yaw_dps = _sh.PIDS[YAW_RATE_DPS].update_spi(yaw_rate_dps, ahrs_data.delta_t);
    outputs.yaw_dps = _sh.output_filters[FD_YAW].filter(outputs.yaw_dps);
#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    const time_us32_t time4 = time_us();
    _sh.time_checks_microseconds[3] = time4 - time3;
#endif

    // The MotorMixerTask is waiting on the message queue, so signal it that there is output data available.
    // This will result in MotorMixer::output_to_motors() being called by the scheduler.
    motor_mixer_message_queue.SIGNAL(outputs);
}
