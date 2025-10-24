#include "Debug.h"
#include "FlightController.h"
#include <AHRS.h>

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
void FlightController::calculateDMaxMultipliers()
{
#if defined(USE_D_MAX)
    for (size_t ii = 0; ii <= RP_AXIS_COUNT; ++ii) {
        _ahM.dMaxMultiplier[ii] = 1.0F;
        if (_dMax.percent[ii] > 1.0F) {
            const float deltaT = _ahrs.getTaskIntervalSeconds();
            const float gyroDeltaD = deltaT * _sh.PIDS[ii].getErrorD(); //!!TODO: check using PID error in D_MAX, surely this is too easy
            const float gyroFactor = std::fabs(_sh.dMaxRangeFilters[ii].filter(gyroDeltaD)) * _dMax.gyroGain;
            const float setpointFactor = std::fabs(_sh.PIDS[ii].getSetpointDelta()) * _dMax.setpointGain;
            const float boost = std::fmaxf(gyroFactor, setpointFactor);
            // boost starts at zero, and by 1.0 we get Dmax, but it can exceed 1.0
            _ahM.dMaxMultiplier[ii] += (_dMax.percent[ii] - 1.0F) * boost;
            _ahM.dMaxMultiplier[ii] = _sh.dMaxLowpassFilters[ii].filter(_ahM.dMaxMultiplier[ii]);
            // limit the multiplier to _dMax.percent
            _ahM.dMaxMultiplier[ii] = std::fminf(_ahM.dMaxMultiplier[ii], _dMax.percent[ii]);
            if (_debug.getMode() == DEBUG_D_MAX) {
                if (ii == FD_ROLL) {
                    _debug.set(DEBUG_D_MAX, 0, lrintf(gyroFactor * 100.0F));
                    _debug.set(DEBUG_D_MAX, 1, lrintf(setpointFactor * 100.0F));
                    _debug.set(DEBUG_D_MAX, 2, lrintf(_fcC.pidConstants[ROLL_RATE_DPS].kd * _ahM.dMaxMultiplier[ROLL_RATE_DPS] * 10));
                } else {
                    _debug.set(DEBUG_D_MAX, 3, lrintf(_fcC.pidConstants[PITCH_RATE_DPS].kd * _ahM.dMaxMultiplier[PITCH_RATE_DPS] * 10));
                }
            }
        }
    }
#endif
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK
*/
void FlightController::recoverFromYawSpin(const xyz_t& gyroENU_RPS, float deltaT)
{
#if defined(USE_YAW_SPIN_RECOVERY)
    if (fabsf(gyroENU_RPS.z) > _yawSpin.recoveredRPS) {
        _sh.outputThrottle = 0.5F; // half throttle gives maximum yaw authority, since outputs will have maximum range before being clipped
        // use the YAW_RATE_DPS PID to bring the spin down to zero
        _sh.PIDS[YAW_RATE_DPS].setSetpoint(0.0F);
        const float yawRateDPS = yawRateNED_DPS(gyroENU_RPS);
        _ahM.outputs[YAW_RATE_DPS] = _sh.PIDS[YAW_RATE_DPS].update(yawRateDPS, deltaT);

        if (fabsf(gyroENU_RPS.z) > _yawSpin.partiallyRecoveredRPS) {
            // we are at a high spin rate, so don't yet attempt to recover the roll and pitch spin
            _ahM.outputs[ROLL_RATE_DPS] = 0.0F;
            _ahM.outputs[PITCH_RATE_DPS] = 0.0F;
        } else {
            // we have partially recovered from the spin, so try and also correct any roll and pitch spin
            _sh.PIDS[ROLL_RATE_DPS].setSetpoint(0.0F);
            const float rollRateDPS = rollRateNED_DPS(gyroENU_RPS);
            _ahM.outputs[ROLL_RATE_DPS] = _sh.PIDS[ROLL_RATE_DPS].update(rollRateDPS, deltaT);

            _sh.PIDS[PITCH_RATE_DPS].setSetpoint(0.0F);
            const float pitchRateDPS = pitchRateNED_DPS(gyroENU_RPS);
            _ahM.outputs[PITCH_RATE_DPS] = _sh.PIDS[PITCH_RATE_DPS].update(pitchRateDPS, deltaT);
        }
    } else {
        // come out of yaw spin recovery
        _sh.yawSpinRecovery = false;
        // switch PID integration back on
        switchPID_integrationOn();
    }
#else
    (void)gyroENU_RPS;
    (void)deltaT;
#endif
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK

In angle mode, the roll and pitch angles are used to set the setpoints for the rollRate and pitchRate PIDs.
In NFE(Not Fast Enough) Racer mode (aka level race mode) is equivalent to angle mode on roll and acro mode on pitch.
*/
void FlightController::updateRateSetpointsForAngleMode(const Quaternion& orientationENU, float deltaT) // NOLINT(readability-make-member-function-const)
{

    // convert orientationENU from the ENU coordinate frame to the NED coordinate frame
    //static const Quaternion qENUtoNED(0.0F, sqrtf(0.5F), sqrtf(0.5F), 0.0F);
    //const Quaternion orientationNED = qENUtoNED * orientationENU;
    //_ahM.rollAngleDegreesRaw = orientationNED.calculateRollDegrees();
    //_ahM.pitchAngleDegreesRaw = orientationNED.calculatePitchDegrees();

    // use the outputs from the "ANGLE" PIDS as the setpoints for the "RATE" PIDs.
    //!!TODO: need to mix in YAW to roll and pitch changes to coordinate turn

    const float yawRateSetpointDPS = _sh.PIDS[YAW_RATE_DPS].getSetpoint();

    // Running the angle PIDs in "quaternion space" rather than "angle space",
    // avoids the computationally expensive Quaternion::calculateRoll and Quaternion::calculatePitch

    //!!TODO: look at using vector product here
    if (_ahM.angleModeCalculationState == ah_t::STATE_CALCULATE_ROLL) {
        if (!_flightModeConfig.level_race_mode) {
            // in level race mode we use angle mode on roll, acro mode on pitch
            // so we only advance calculation to pitch if not in level race mode
            _ahM.angleModeCalculationState = ah_t::STATE_CALCULATE_PITCH;
        }

        _ahM.rollSinAngle = -orientationENU.sinRollClipped(); // sin(x-180) = -sin(x)
        if (_useQuaternionSpaceForAngleMode) {
            const float rollSinAngleDelta = _sh.dTermFilters1[ROLL_SIN_ANGLE].filter(_ahM.rollSinAngle - _sh.PIDS[ROLL_SIN_ANGLE].getPreviousMeasurement());
            _ahM.outputs[ROLL_SIN_ANGLE] = _sh.PIDS[ROLL_SIN_ANGLE].updateDelta(_ahM.rollSinAngle, rollSinAngleDelta, deltaT);
            _ahM.rollRateSetpointDPS = _ahM.outputs[ROLL_SIN_ANGLE];
        } else {
            _ahM.rollAngleDegreesRaw = orientationENU.calculateRollDegrees() - 180.0F;
            const float rollAngleDelta = _sh.dTermFilters1[ROLL_ANGLE_DEGREES].filter(_ahM.rollAngleDegreesRaw - _sh.PIDS[ROLL_ANGLE_DEGREES].getPreviousMeasurement());
            _ahM.outputs[ROLL_ANGLE_DEGREES] = _sh.PIDS[ROLL_ANGLE_DEGREES].updateDelta(_ahM.rollAngleDegreesRaw, rollAngleDelta, deltaT) * _maxRollRateDPS;
            _ahM.rollRateSetpointDPS = _ahM.outputs[ROLL_ANGLE_DEGREES];
        }
        // a component of YAW changes roll, so update accordingly !!TODO:check sign
        _ahM.rollRateSetpointDPS -= yawRateSetpointDPS * _ahM.rollSinAngle;
        _sh.PIDS[ROLL_RATE_DPS].setSetpoint(_ahM.rollRateSetpointDPS);
    } else {
        _ahM.angleModeCalculationState = ah_t::STATE_CALCULATE_ROLL;

        _ahM.pitchSinAngle = -orientationENU.sinPitchClipped(); // this is cheaper to calculate than sinRoll
        if (_useQuaternionSpaceForAngleMode) {
            const float pitchSinAngleDelta = _sh.dTermFilters1[PITCH_SIN_ANGLE].filter(_ahM.pitchSinAngle - _sh.PIDS[PITCH_SIN_ANGLE].getPreviousMeasurement());
            _ahM.outputs[PITCH_SIN_ANGLE] = _sh.PIDS[PITCH_SIN_ANGLE].updateDelta(_ahM.pitchSinAngle, pitchSinAngleDelta, deltaT);
            _ahM.pitchRateSetpointDPS = _ahM.outputs[PITCH_SIN_ANGLE];
        } else {
            _ahM.pitchAngleDegreesRaw = -orientationENU.calculatePitchDegrees();
            const float pitchAngleDelta = _sh.dTermFilters1[ROLL_ANGLE_DEGREES].filter(_ahM.pitchAngleDegreesRaw - _sh.PIDS[PITCH_ANGLE_DEGREES].getPreviousMeasurement());
            _ahM.outputs[PITCH_ANGLE_DEGREES] = _sh.PIDS[PITCH_ANGLE_DEGREES].updateDelta(_ahM.pitchAngleDegreesRaw, pitchAngleDelta, deltaT) * _maxPitchRateDPS;
            _ahM.pitchRateSetpointDPS = _ahM.outputs[PITCH_ANGLE_DEGREES];
        }
        // a component of YAW changes roll, so update accordingly !!TODO:check sign
        _ahM.pitchRateSetpointDPS += yawRateSetpointDPS * _ahM.pitchSinAngle;
        _sh.PIDS[PITCH_RATE_DPS].setSetpoint(_ahM.pitchRateSetpointDPS);
    }


    // the cosRoll and cosPitch functions are reasonably cheap, they both involve taking a square root
    // both are positive in ANGLE mode, since absolute values of both roll and pitch angles are less than 90 degrees
#if false
    const float rollCosAngle = orientationENU.cosRoll();
    const float pitchCosAngle = orientationENU.cosPitch();
    const float yawRateSetpointAttenuation = fmaxf(rollCosAngle, pitchCosAngle);
#else
    const float rollSinAngle2 = _ahM.rollSinAngle*_ahM.rollSinAngle;
    const float pitchSinAngle2 = _ahM.pitchSinAngle*_ahM.pitchSinAngle;
    const float minSinAngle2 = fminf(rollSinAngle2, pitchSinAngle2);
    const float yawRateSetpointAttenuation = sqrtf(1.0F - minSinAngle2); // this is equal to fmaxf(rollCosAngle, pitchCosAngle)
#endif
    // attenuate yaw rate setpoint
    _sh.PIDS[YAW_RATE_DPS].setSetpoint(_rxC.yawRateSetpointDPS*yawRateSetpointAttenuation);
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK
*/
float FlightController::calculateITermError(size_t axis, float measurement)
{
    const float setpoint = _sh.PIDS[axis].getSetpoint();
    // iTermError is just `setpoint - measurement`, if there is no ITerm relax
    float iTermError = setpoint - measurement;
#if defined(USE_ITERM_RELAX)
    if (_iTermRelaxConfig.iterm_relax == ITERM_RELAX_ON) {
        const float setpointLp = _rxC.setpointLPs[axis];
        const float setpointHp = fabsf(setpoint - setpointLp);
        float setpointThresholdDPS = _iTermRelax.setpointThresholdDPS;
        if (_rxC.useAngleMode) {
            setpointThresholdDPS *= 0.2F;
        }
        const float itermRelaxFactor = std::fmaxf(0.0F, 1.0F - setpointHp/setpointThresholdDPS);
        iTermError *= itermRelaxFactor;

        if (axis == ROLL_RATE_DPS && _debug.getMode() == DEBUG_ITERM_RELAX) {
            _debug.set(DEBUG_ITERM_RELAX, 0, lrintf(setpointHp));
            _debug.set(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor));
            _debug.set(DEBUG_ITERM_RELAX, 2, lrintf(iTermError));
        }
    }
#endif
    return iTermError;
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK
It is typically called at frequency of between 1000Hz and 8000Hz, so it has to be FAST.

The FlightController uses the NED (North-East-Down) coordinate convention.
gyroRPS, acc, and orientation come from the AHRS and use the ENU (East-North-Up) coordinate convention.
*/
void FlightController::updateOutputsUsingPIDs(const xyz_t& gyroENU_RPS, const xyz_t& accENU, const Quaternion& orientationENU, float deltaT)
{
    (void)accENU; // not using acc, since we use the orientation quaternion instead

#if defined(USE_YAW_SPIN_RECOVERY)
    if (_sh.yawSpinRecovery) {
        recoverFromYawSpin(gyroENU_RPS, deltaT);
        const VehicleControllerMessageQueue::queue_item_t queueItem {
            .throttle = _sh.outputThrottle,
            .roll = _ahM.outputs[ROLL_RATE_DPS],
            .pitch = _ahM.outputs[PITCH_RATE_DPS],
            .yaw = _ahM.outputs[YAW_RATE_DPS]
        };
        SIGNAL(queueItem);
        return;
    }
#endif

#if defined(USE_D_MAX)
    calculateDMaxMultipliers();
#endif

    if (_rxC.useAngleMode) {
        updateRateSetpointsForAngleMode(orientationENU, deltaT);
    }

    // Use the PIDs to calculate the outputs for each axis.
    // Note that the delta-values (ie the DTerms) are filtered:
    // this is because they are especially noisy, being the derivative of a noisy value.

    //
    // Roll axis
    //
    const float rollRateDPS = rollRateNED_DPS(gyroENU_RPS);
    float rollRateDeltaFilteredDPS = _sh.dTermFilters1[ROLL_RATE_DPS].filter(rollRateDPS - _sh.PIDS[ROLL_RATE_DPS].getPreviousMeasurement());
    rollRateDeltaFilteredDPS = _sh.dTermFilters2[ROLL_RATE_DPS].filter(rollRateDeltaFilteredDPS);
    _ahM.outputs[ROLL_RATE_DPS] = _sh.PIDS[ROLL_RATE_DPS].updateDeltaITerm(
                                                                rollRateDPS, 
                                                                rollRateDeltaFilteredDPS * _rxC.TPA * _ahM.dMaxMultiplier[ROLL_RATE_DPS], 
                                                                calculateITermError(ROLL_RATE_DPS, rollRateDPS),
                                                                deltaT);
    // filter the output
    _ahM.outputs[ROLL_RATE_DPS] = _sh.outputFilters[ROLL_RATE_DPS].filter(_ahM.outputs[ROLL_RATE_DPS]);

    //
    // Pitch axis
    //
    const float pitchRateDPS = pitchRateNED_DPS(gyroENU_RPS);
    float pitchRateDeltaFilteredDPS = _sh.dTermFilters1[PITCH_RATE_DPS].filter(rollRateDPS - _sh.PIDS[PITCH_RATE_DPS].getPreviousMeasurement());
    pitchRateDeltaFilteredDPS = _sh.dTermFilters2[PITCH_RATE_DPS].filter(pitchRateDeltaFilteredDPS);
    _ahM.outputs[PITCH_RATE_DPS] = _sh.PIDS[PITCH_RATE_DPS].updateDeltaITerm(
                                                                pitchRateDPS,
                                                                pitchRateDeltaFilteredDPS * _rxC.TPA * _ahM.dMaxMultiplier[PITCH_RATE_DPS],
                                                                calculateITermError(PITCH_RATE_DPS, pitchRateDPS),
                                                                deltaT);
    // filter the output
    _ahM.outputs[PITCH_RATE_DPS] = _sh.outputFilters[PITCH_RATE_DPS].filter(_ahM.outputs[PITCH_RATE_DPS]);

    //
    // Yaw axis
    //
    // DTerm is zero for yawRate, so call updatePIS() with no DTerm filtering, no TPA, no DMax, no ITerm relax, and no KTerm
    const float yawRateDPS = yawRateNED_DPS(gyroENU_RPS);
    _ahM.outputs[YAW_RATE_DPS] = _sh.PIDS[YAW_RATE_DPS].updateSPI(yawRateDPS, deltaT);
    // filter the output
    _ahM.outputs[YAW_RATE_DPS] = _sh.outputFilters[YAW_RATE_DPS].filter(_ahM.outputs[YAW_RATE_DPS]);

    // The FlightControllerTask is waiting on the message queue, so signal it that there is output data available.
    // This will result in outputToMixer() being called by the scheduler.
    const VehicleControllerMessageQueue::queue_item_t queueItem {
        .throttle = _sh.outputThrottle,
        .roll = _ahM.outputs[ROLL_RATE_DPS],
        .pitch = _ahM.outputs[PITCH_RATE_DPS],
        .yaw = _ahM.outputs[YAW_RATE_DPS]
    };
    SIGNAL(queueItem);
}
