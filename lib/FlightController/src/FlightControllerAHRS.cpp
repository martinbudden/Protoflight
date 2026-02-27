#include "FlightController.h"

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

Calculate the dMaxMultipliers.

This are multipliers that are applied to the roll and pitch axis DTerms.
This means DTerms can be low in normal flight but are boosted to a higher value when required.
They are boosted when the DTerm error is small and the setpoint change is also small.
*/
void FlightController::calculateDMaxMultipliers(Debug& debug)
{
#if defined(USE_D_MAX)
#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{RP_AXIS_COUNT})) {
#else
    for (size_t ii = 0; ii <= RP_AXIS_COUNT; ++ii) {
#endif
        _ahM.dMaxMultiplier[ii] = 1.0F;
        if (_dMax.percent[ii] > 1.0F) {
            const float deltaT = static_cast<float>(_task_interval_microseconds) * 0.000001F;
            const float gyroDeltaD = deltaT * _sh.PIDS[ii].get_error_d(); //!!TODO: check using PID error in D_MAX, surely this is too easy
            const float gyroFactor = std::fabs(_sh.dMaxRangeFilters[ii].filter(gyroDeltaD)) * _dMax.gyroGain;
            const float setpointFactor = std::fabs(_sh.PIDS[ii].get_setpoint_delta()) * _dMax.setpointGain;
            const float boost = std::fmaxf(gyroFactor, setpointFactor);
            // boost starts at zero, and by 1.0 we get Dmax, but it can exceed 1.0
            _ahM.dMaxMultiplier[ii] += (_dMax.percent[ii] - 1.0F) * boost;
            _ahM.dMaxMultiplier[ii] = _sh.dMaxLowpassFilters[ii].filter(_ahM.dMaxMultiplier[ii]);
            // limit the multiplier to _dMax.percent
            _ahM.dMaxMultiplier[ii] = std::fminf(_ahM.dMaxMultiplier[ii], _dMax.percent[ii]);
            if (debug.getMode() == DEBUG_D_MAX) {
                if (ii == FD_ROLL) {
                    debug.set(DEBUG_D_MAX, 0, lrintf(gyroFactor * 100.0F));
                    debug.set(DEBUG_D_MAX, 1, lrintf(setpointFactor * 100.0F));
                    debug.set(DEBUG_D_MAX, 2, lrintf(_fcC.pidConstants[ROLL_RATE_DPS].kd * _ahM.dMaxMultiplier[ROLL_RATE_DPS] * 10));
                } else {
                    debug.set(DEBUG_D_MAX, 3, lrintf(_fcC.pidConstants[PITCH_RATE_DPS].kd * _ahM.dMaxMultiplier[PITCH_RATE_DPS] * 10));
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
void FlightController::applyCrashFlipToMotors(const xyz_t& gyro_rps, float deltaT, MotorMixerMessageQueue& motor_mixer_message_queue)
{
    (void)gyro_rps;
    (void)deltaT;
    (void)motor_mixer_message_queue;
}


/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK
*/
void FlightController::recoverFromYawSpin(const xyz_t& gyro_rps, float deltaT, MotorMixerMessageQueue& motor_mixer_message_queue)
{
#if defined(USE_YAW_SPIN_RECOVERY)
    if (std::fabs(gyro_rps.z) > _yawSpin.recoveredRPS) {
        _sh.outputThrottle = 0.5F; // half throttle gives maximum yaw authority, since outputs will have maximum range before being clipped
        // use the YAW_RATE_DPS PID to bring the spin down to zero
        _sh.PIDS[YAW_RATE_DPS].set_setpoint(0.0F);
        const float yawRateDPS = yawRateNED_DPS(gyro_rps);
        motor_mixer_message_queue_item_t outputs {
            .throttle = _sh.outputThrottle,
            .roll_dps = 0.0F,
            .pitch_dps = 0.0F,
            .yaw_dps = _sh.PIDS[YAW_RATE_DPS].update(yawRateDPS, deltaT)
        };
        if (std::fabs(gyro_rps.z) <= _yawSpin.partiallyRecoveredRPS) {
            // we have partially recovered from the spin, so try and also correct any roll and pitch spin
            _sh.PIDS[ROLL_RATE_DPS].set_setpoint(0.0F);
            const float rollRateDPS = rollRateNED_DPS(gyro_rps);
            outputs.roll_dps = _sh.PIDS[ROLL_RATE_DPS].update(rollRateDPS, deltaT);

            _sh.PIDS[PITCH_RATE_DPS].set_setpoint(0.0F);
            const float pitchRateDPS = pitchRateNED_DPS(gyro_rps);
            outputs.pitch_dps = _sh.PIDS[PITCH_RATE_DPS].update(pitchRateDPS, deltaT);
        }
        motor_mixer_message_queue.SIGNAL(outputs);
    } else {
        // come out of yaw spin recovery
        _sh.yawSpinRecovery = false;
        // switch PID integration back on
        switchPID_integrationOn();
    }
#else
    (void)gyro_rps;
    (void)deltaT;
    (void)motor_mixer_message_queue;
#endif
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK

In angle mode, the roll and pitch angles are used to set the setpoints for the rollRate and pitchRate PIDs.
Level Race Mode (aka NFE(Not Fast Enough) mode) is equivalent to angle mode on roll and acro mode on pitch.
*/
void FlightController::updateRateSetpointsForAngleMode(const Quaternion& orientation, float deltaT) // NOLINT(readability-make-member-function-const)
{
    // convert orientationENU from the ENU coordinate frame to the NED coordinate frame
    //static const Quaternion qENUtoNED(0.0F, sqrtf(0.5F), sqrtf(0.5F), 0.0F);
    //const Quaternion orientationNED = qENUtoNED * orientationENU;
    //_ahM.rollAngleDegreesRaw = orientationNED.calculateRollDegrees();
    //_ahM.pitchAngleDegreesRaw = orientationNED.calculatePitchDegrees();

    // use the outputs from the "ANGLE" PIDS as the setpoints for the "RATE" PIDs.
    //!!TODO: need to mix in YAW to roll and pitch changes to coordinate turn

    //const float yawRateSetpointDPS = _sh.PIDS[YAW_RATE_DPS].get_setpoint();

    // Running the angle PIDs in "quaternion space" rather than "angle space",
    // avoids the computationally expensive Quaternion::calculateRoll and Quaternion::calculatePitch

    //!!TODO: look at using vector product here
    if (_ahM.amcs.state == STATE_CALCULATE_ROLL) {
        if (!_rxC.useLevelRaceMode) {
            // in level race mode we use angle mode on roll, acro mode on pitch
            // so we only advance calculation to pitch if not in level race mode
            _ahM.amcs.state = STATE_CALCULATE_PITCH;
        }

        _ahM.amcs.rollSinAngle = rollSinAngleNED(orientation);
        float rollRateSetpointDPS;
#if defined(USE_SIN_ANGLE_PIDS)
        if (_useQuaternionSpaceForAngleMode) {
            const float rollSinAngleDelta = _sh.dTermFilters1[ROLL_SIN_ANGLE].filter(_ahM.amcs.rollSinAngle - _sh.PIDS[ROLL_SIN_ANGLE].get_previous_measurement());
            rollRateSetpointDPS = _sh.PIDS[ROLL_SIN_ANGLE].update_delta(_ahM.amcs.rollSinAngle, rollSinAngleDelta, deltaT);
        } else
#endif
        {
            const float rollAngleDegrees = rollAngleDegreesNED(orientation);
            const float rollAngleDelta = _sh.dTermFilters1[ROLL_ANGLE_DEGREES].filter(rollAngleDegrees - _sh.PIDS[ROLL_ANGLE_DEGREES].get_previous_measurement());
            // calculate roll rate setpoint in degrees, range is [-_maxRollAngleDegrees, _maxRollAngleDegrees], typically [-60, 60]
            const float rollRateSetpointDegrees = _sh.PIDS[ROLL_ANGLE_DEGREES].update_delta(rollAngleDegrees, rollAngleDelta, deltaT);
            // convert to value in range [-1.0, 1.0] to be used for the ROLL_RATE_DPS setpoint
            rollRateSetpointDPS = std::clamp(rollRateSetpointDegrees / _maxRollAngleDegrees, -1.0F, 1.0F) * _maxRollRateDPS;
        }
        // a component of YAW changes roll, so update accordingly !!TODO:check sign
        //rollRateSetpointDPS -= yawRateSetpointDPS * _ahM.amcs.rollSinAngle;
        _sh.PIDS[ROLL_RATE_DPS].set_setpoint(rollRateSetpointDPS);
    } else {
        _ahM.amcs.state = STATE_CALCULATE_ROLL;

        _ahM.amcs.pitchSinAngle = pitchSinAngleNED(orientation);
        float pitchRateSetpointDPS;
#if defined(USE_SIN_ANGLE_PIDS)
        if (_useQuaternionSpaceForAngleMode) {
            const float pitchSinAngleDelta = _sh.dTermFilters1[PITCH_SIN_ANGLE].filter(_ahM.amcs.pitchSinAngle - _sh.PIDS[PITCH_SIN_ANGLE].get_previous_measurement());
            pitchRateSetpointDPS = _sh.PIDS[PITCH_SIN_ANGLE].update_delta(_ahM.amcs.pitchSinAngle, pitchSinAngleDelta, deltaT);
        } else
#endif
        {
            const float pitchAngleDegrees = pitchAngleDegreesNED(orientation);
            const float pitchAngleDelta = _sh.dTermFilters1[PITCH_ANGLE_DEGREES].filter(pitchAngleDegrees - _sh.PIDS[PITCH_ANGLE_DEGREES].get_previous_measurement());
            // calculate pitch rate setpoint in degrees, range is [-_maxPitchAngleDegrees, _maxPitchAngleDegrees], typically [-60, 60]
            const float pitchRateSetpointDegrees = _sh.PIDS[PITCH_ANGLE_DEGREES].update_delta(pitchAngleDegrees, pitchAngleDelta, deltaT);
            // convert to value in range [-1.0, 1.0] to be used for the PITCH_RATE_DPS setpoint
            pitchRateSetpointDPS = std::clamp(pitchRateSetpointDegrees / _maxPitchAngleDegrees, -1.0F, 1.0F) * _maxPitchRateDPS;
        }
        // a component of YAW changes pitch, so update accordingly !!TODO:check sign
        //pitchRateSetpoint += yawRateSetpointDPS * _ahM.amcs.pitchSinAngle;
        _sh.PIDS[PITCH_RATE_DPS].set_setpoint(pitchRateSetpointDPS);
    }

    // the cosRoll and cosPitch functions are reasonably cheap, they both involve taking a square root
    // both are positive in ANGLE mode, since absolute values of both roll and pitch angles are less than 90 degrees
#if false
    const float rollCosAngle = rollCosAngleNED(orientation);
    const float pitchCosAngle = pitchCosAngleNED(orientation);
    const float yawRateSetpointAttenuation = fmaxf(rollCosAngle, pitchCosAngle);
#else
    const float rollSinAngleSquared = _ahM.amcs.rollSinAngle*_ahM.amcs.rollSinAngle;
    const float pitchSinAngleSquared = _ahM.amcs.pitchSinAngle*_ahM.amcs.pitchSinAngle;
    const float minSinAngleSquared = fminf(rollSinAngleSquared, pitchSinAngleSquared);
    const float yawRateSetpointAttenuation = sqrtf(1.0F - minSinAngleSquared); // this is equal to fmaxf(rollCosAngle, pitchCosAngle)
#endif
    // attenuate yaw rate setpoint
    _sh.PIDS[YAW_RATE_DPS].set_setpoint(_rxC.yawRateSetpointDPS*yawRateSetpointAttenuation);
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK
*/
float FlightController::calculateITermError(size_t axis, float measurement, Debug& debug)
{
    const float setpoint = _sh.PIDS[axis].get_setpoint();
    // iTermError is just `setpoint - measurement`, if there is no ITerm relax
    float iTermError = setpoint - measurement;
#if defined(USE_ITERM_RELAX)
    if (_iTermRelaxConfig.iterm_relax == ITERM_RELAX_ON) {
        const float setpointLp = _rxC.setpointLPs[axis];
        const float setpointHp = std::fabs(setpoint - setpointLp);
        float setpointThresholdDPS = _iTermRelax.setpointThresholdDPS;
        if (_rxC.useAngleMode) {
            setpointThresholdDPS *= 0.2F;
        }
        const float itermRelaxFactor = std::fmaxf(0.0F, 1.0F - setpointHp/setpointThresholdDPS);
        iTermError *= itermRelaxFactor;

        if (axis == ROLL_RATE_DPS && debug.getMode() == DEBUG_ITERM_RELAX) {
            debug.set(DEBUG_ITERM_RELAX, 0, lrintf(setpointHp));
            debug.set(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor));
            debug.set(DEBUG_ITERM_RELAX, 2, lrintf(iTermError));
        }
    }
#else
    (void)debug;
#endif
    return iTermError;
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK
It is typically called at frequency of between 1000Hz and 8000Hz, so it has to be FAST.

The FlightController uses the NED (North-East-Down) coordinate convention.
gyro_rps, acc, and orientation come from the AHRS and use the ENU (East-North-Up) coordinate convention.
*/
void FlightController::update_outputs_using_pids(const ahrs_data_t& ahrsData, AhrsMessageQueue& ahrsMessageQueue, MotorMixerMessageQueue& motor_mixer_message_queue, Debug& debug)
{
#if defined(USE_BLACKBOX) || defined(USE_BACKCHANNEL) || defined(USE_DASHBOARD)
    // signalling/sending to the message queue is not free, so, in this time-critical function, we only do it if necessary
    ++_ahM.sendBlackboxMessageCount;
    if (_ahM.sendBlackboxMessageCount >= _sendBlackboxMessageDenominator) {
        _ahM.sendBlackboxMessageCount = 0;
        if (_sh.blackboxActive) {
            ahrsMessageQueue.SIGNAL(ahrsData);
        }
#if defined(USE_BACKCHANNEL) || defined(USE_DASHBOARD)
        ahrsMessageQueue.SEND_AHRS_DATA(ahrsData);
#endif
    }
#else
    (void)ahrsMessageQueue;
#endif
    if (_sh.crashFlipModeActive) {
        applyCrashFlipToMotors(ahrsData.acc_gyro_rps.gyro_rps, ahrsData.delta_t, motor_mixer_message_queue);
        return;
    }

#if defined(USE_YAW_SPIN_RECOVERY)
    if (_sh.yawSpinRecovery) {
        recoverFromYawSpin(ahrsData.acc_gyro_rps.gyro_rps, ahrsData.delta_t, motor_mixer_message_queue);
        return;
    }
#endif
#if defined(USE_D_MAX)
    calculateDMaxMultipliers(debug);
#endif

#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    const time_us32_t time0 = time_us();
#endif
#if defined(USE_ANGLE_MODE_LOCKED_ON)
    updateRateSetpointsForAngleMode(ahrsData.orientation, ahrsData.delta_t);
#else
    if (_rxC.useAngleMode) {
        updateRateSetpointsForAngleMode(ahrsData.orientation, ahrsData.delta_t);
    }
#endif
#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    const time_us32_t time1 = time_us();
    _sh.timeChecksMicroseconds[0] = time1 - time0;
#endif

    motor_mixer_message_queue_item_t outputs {}; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    outputs.throttle = _sh.outputThrottle;

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
    const float rollRateDPS = rollRateNED_DPS(ahrsData.acc_gyro_rps.gyro_rps);
    // filter the DTerm twice
    float rollRateDeltaFilteredDPS = _sh.dTermFilters1[ROLL_RATE_DPS].filter(rollRateDPS - _sh.PIDS[ROLL_RATE_DPS].get_previous_measurement());
    rollRateDeltaFilteredDPS = _sh.dTermFilters2[ROLL_RATE_DPS].filter(rollRateDeltaFilteredDPS);
    outputs.roll_dps = _sh.PIDS[ROLL_RATE_DPS].update_delta_iterm(
                                                    rollRateDPS,
                                                    rollRateDeltaFilteredDPS * _rxC.TPA * _ahM.dMaxMultiplier[ROLL_RATE_DPS],
                                                    calculateITermError(ROLL_RATE_DPS, rollRateDPS, debug),
                                                    ahrsData.delta_t);
    outputs.roll_dps = _sh.outputFilters[FD_ROLL].filter(outputs.roll_dps);
#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    const time_us32_t time2 = time_us();
    _sh.timeChecksMicroseconds[1] = time2 - time1;
#endif

    //
    // Pitch axis
    //
    const float pitchRateDPS = pitchRateNED_DPS(ahrsData.acc_gyro_rps.gyro_rps);
    // filter the DTerm twice
    float pitchRateDeltaFilteredDPS = _sh.dTermFilters1[PITCH_RATE_DPS].filter(rollRateDPS - _sh.PIDS[PITCH_RATE_DPS].get_previous_measurement());
    pitchRateDeltaFilteredDPS = _sh.dTermFilters2[PITCH_RATE_DPS].filter(pitchRateDeltaFilteredDPS);
    outputs.pitch_dps = _sh.PIDS[PITCH_RATE_DPS].update_delta_iterm(
                                                    pitchRateDPS,
                                                    pitchRateDeltaFilteredDPS * _rxC.TPA * _ahM.dMaxMultiplier[PITCH_RATE_DPS],
                                                    calculateITermError(PITCH_RATE_DPS, pitchRateDPS, debug),
                                                    ahrsData.delta_t);
    outputs.pitch_dps = _sh.outputFilters[FD_PITCH].filter(outputs.pitch_dps);
#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    const time_us32_t time3 = time_us();
    _sh.timeChecksMicroseconds[2] = time3 - time2;
#endif


    //
    // Yaw axis
    //
    // DTerm is zero for yawRate, so call updateSPI() with no DTerm filtering, no TPA, no DMax, no ITerm relax, and no KTerm
    const float yawRateDPS = yawRateNED_DPS(ahrsData.acc_gyro_rps.gyro_rps);
    outputs.yaw_dps = _sh.PIDS[YAW_RATE_DPS].update_spi(yawRateDPS, ahrsData.delta_t);
    outputs.yaw_dps = _sh.outputFilters[FD_YAW].filter(outputs.yaw_dps);
#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    const time_us32_t time4 = time_us();
    _sh.timeChecksMicroseconds[3] = time4 - time3;
#endif

    // The MotorMixerTask is waiting on the message queue, so signal it that there is output data available.
    // This will result in MotorMixer::output_to_motors() being called by the scheduler.
    motor_mixer_message_queue.SIGNAL(outputs);
}
