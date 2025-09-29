#include "Debug.h"
#include "FlightController.h"
#include <AHRS.h>


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
    for (size_t ii = ROLL_RATE_DPS; ii <= PITCH_RATE_DPS; ++ii) {
        _ahM.dMaxMultiplier[ii] = 1.0F;
        if (_dMax.percent[ii] > 1.0F) {
            const float deltaT = _ahrs.getTaskIntervalSeconds();
            const float gyroDeltaD = deltaT * _sh.PIDS[ii].getErrorD(); //!!TODO: check using PID error in D_MAX, surely this is too easy
            const float gyroFactor = std::fabs(_sh.dMaxRangeFilter[ii].filter(gyroDeltaD)) * _dMax.gyroGain;
            const float setpointFactor = std::fabs(_sh.PIDS[ii].getSetpointDelta()) * _dMax.setpointGain;
            const float boost = std::fmaxf(gyroFactor, setpointFactor);
            // dMaxBoost starts at zero, and by 1.0 we get Dmax, but it can exceed 1.0
            _ahM.dMaxMultiplier[ii] += (_dMax.percent[ii] - 1.0F) * boost;
            _ahM.dMaxMultiplier[ii] = _sh.dMaxLowpassFilter[ii].filter(_ahM.dMaxMultiplier[ii]);
            // limit the multiplier to _dMaxPercent
            _ahM.dMaxMultiplier[ii] = std::fminf(_ahM.dMaxMultiplier[ii], _dMax.percent[ii]);
            if (_debug.getMode() == DEBUG_D_MAX) {
                if (ii == FD_ROLL) {
                    _debug.set(DEBUG_D_MAX, 0, lrintf(gyroFactor * 100));
                    _debug.set(DEBUG_D_MAX, 1, lrintf(setpointFactor * 100));
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
    if (fabsf(gyroENU_RPS.z) > _yawSpinRecoveredRPS) {
        _sh.outputThrottle = 0.5F; // half throttle gives maximum yaw authority, since outputs will have maximum range before being clipped
        // use the YAW_RATE_DPS PID to bring the spin down to zero
        _sh.PIDS[YAW_RATE_DPS].setSetpoint(0.0F);
        const float yawRateDPS = yawRateNED_DPS(gyroENU_RPS);
        _ahM.outputs[YAW_RATE_DPS] = _sh.PIDS[YAW_RATE_DPS].update(yawRateDPS, deltaT);

        if (fabsf(gyroENU_RPS.z) > _yawSpinPartiallyRecoveredRPS) {
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
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK

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

    const float yawRateSetpointDPS = _sh.PIDS[YAW_RATE_DPS].getSetpoint();

    if (_angleModeUseQuaternionSpace) {
        // Runs the angle PIDs in "quaternion space" rather than "angle space",
        // avoiding the computationally expensive Quaternion::calculateRoll and Quaternion::calculatePitch
        if (_ahM.angleModeCalculationState == ah_t::STATE_CALCULATE_ROLL) {
            if (!_useAngleModeOnRollAcroModeOnPitch) {
                // don't advance calculation to pitch axis when in level race mode
                _ahM.angleModeCalculationState = ah_t::STATE_CALCULATE_PITCH;
            }
            _ahM.rollSinAngle = -orientationENU.sinRollClipped(); // sin(x-180) = -sin(x)
            const float rollSinAngleDelta = _sh.rollAngleDTermFilter.filter(_ahM.rollSinAngle - _sh.PIDS[ROLL_SIN_ANGLE].getPreviousMeasurement());
            _ahM.outputs[ROLL_SIN_ANGLE] = _sh.PIDS[ROLL_SIN_ANGLE].update(_ahM.rollSinAngle, rollSinAngleDelta, deltaT);
            _ahM.rollRateSetpointDPS = _ahM.outputs[ROLL_SIN_ANGLE];
            // a component of YAW changes roll, so update accordingly !!TODO:check sign
            _ahM.rollRateSetpointDPS -= yawRateSetpointDPS * _ahM.rollSinAngle;
        } else {
            _ahM.angleModeCalculationState = ah_t::STATE_CALCULATE_ROLL;
            _ahM.pitchSinAngle = -orientationENU.sinPitchClipped(); // this is cheaper to calculate than sinRoll
            const float pitchSinAngleDelta = _sh.rollAngleDTermFilter.filter(_ahM.pitchSinAngle - _sh.PIDS[PITCH_SIN_ANGLE].getPreviousMeasurement());
            _ahM.outputs[PITCH_SIN_ANGLE] = _sh.PIDS[PITCH_SIN_ANGLE].update(_ahM.pitchSinAngle, pitchSinAngleDelta, deltaT);
            _ahM.pitchRateSetpointDPS = _ahM.outputs[PITCH_SIN_ANGLE];
            // a component of YAW changes roll, so update accordingly !!TODO:check sign
            _ahM.pitchRateSetpointDPS += yawRateSetpointDPS * _ahM.pitchSinAngle;
        }
    } else {
        //!!TODO: this all needs checking, especially for scale
        //!!TODO: PID constants need to be scaled so these outputs are in the range [-1, 1]
        // calculate roll rate and pitch rate setpoints in the NED coordinate frame
        // this is a computationally expensive calculation, so alternate between roll and pitch each time this function is called
        if (_ahM.angleModeCalculationState == ah_t::STATE_CALCULATE_ROLL) {
            if (!_useAngleModeOnRollAcroModeOnPitch) {
                // don't advance calculation to pitch axis when in level race mode
                _ahM.angleModeCalculationState = ah_t::STATE_CALCULATE_PITCH;
            }
            _ahM.rollSinAngle = -orientationENU.sinRoll(); // sin(x-180) = -sin(x)
            _rollAngleDegreesRaw = orientationENU.calculateRollDegrees() - 180.0F;
            const float rollAngleDelta = _sh.rollAngleDTermFilter.filter(_rollAngleDegreesRaw - _sh.PIDS[ROLL_ANGLE_DEGREES].getPreviousMeasurement());
            _ahM.outputs[ROLL_ANGLE_DEGREES] = _sh.PIDS[ROLL_ANGLE_DEGREES].update(_rollAngleDegreesRaw, rollAngleDelta, deltaT) * _maxRollRateDPS;
            _ahM.rollRateSetpointDPS = _ahM.outputs[ROLL_ANGLE_DEGREES];
            _ahM.rollRateSetpointDPS -= yawRateSetpointDPS * _ahM.rollSinAngle;
        } else {
            _ahM.angleModeCalculationState = ah_t::STATE_CALCULATE_ROLL;
            _ahM.pitchSinAngle = -orientationENU.sinPitch(); // this is cheaper to calculate than sinRoll
            _pitchAngleDegreesRaw = -orientationENU.calculatePitchDegrees();
            const float pitchAngleDelta = _sh.pitchAngleDTermFilter.filter(_pitchAngleDegreesRaw - _sh.PIDS[PITCH_ANGLE_DEGREES].getPreviousMeasurement());
            _ahM.outputs[PITCH_ANGLE_DEGREES] = _sh.PIDS[PITCH_ANGLE_DEGREES].update(_pitchAngleDegreesRaw, pitchAngleDelta, deltaT) * _maxPitchRateDPS;
            _ahM.pitchRateSetpointDPS = _ahM.outputs[PITCH_ANGLE_DEGREES];
            _ahM.pitchRateSetpointDPS += yawRateSetpointDPS * _ahM.pitchSinAngle;
        }
    }

    // use the outputs from the "ANGLE" PIDS as the setpoints for the "RATE" PIDs.
    //!!TODO: need to mix in YAW to roll and pitch changes to coordinate turn
    _sh.PIDS[ROLL_RATE_DPS].setSetpoint(_ahM.rollRateSetpointDPS);
    _sh.PIDS[PITCH_RATE_DPS].setSetpoint(_ahM.pitchRateSetpointDPS);

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

The FlightController uses the NED (North-East-Down) coordinate convention.
gyroRPS, acc, and orientation come from the AHRS and use the ENU (East-North-Up) coordinate convention.

NOTE: this function is called in the context of the AHRS task.
It is typically called at frequency of between 1000Hz and 8000Hz, so it has to be FAST.
*/
void FlightController::updateOutputsUsingPIDs(const xyz_t& gyroENU_RPS, const xyz_t& accENU, const Quaternion& orientationENU, float deltaT)
{
    (void)accENU; // not using acc, since we use the orientation quaternion instead

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

#if defined(USE_D_MAX)
    calculateDMaxMultipliers();
#endif

    if (_rxC.useAngleMode) {
        updateRateSetpointsForAngleMode(orientationENU, deltaT);
    }

    // Use the PIDs to calculate the outputs for each axis.
    // Note that the delta-values (ie the DTerms) are filtered:
    // this is because they are especially noisy, being the derivative of a noisy value.

    const float rollRateDPS = rollRateNED_DPS(gyroENU_RPS);
    const float rollRateDeltaDPS = _sh.rollRateDTermFilter.filter(rollRateDPS - _sh.PIDS[ROLL_RATE_DPS].getPreviousMeasurement());
    _ahM.outputs[ROLL_RATE_DPS] = _sh.PIDS[ROLL_RATE_DPS].updateDelta(rollRateDPS, rollRateDeltaDPS*_rxC.TPA*_ahM.dMaxMultiplier[ROLL_RATE_DPS], deltaT);
    // filter the output
    _ahM.outputs[ROLL_RATE_DPS] = _sh.outputFilters[ROLL_RATE_DPS].filter(_ahM.outputs[ROLL_RATE_DPS]);

    const float pitchRateDPS = pitchRateNED_DPS(gyroENU_RPS);
    const float pitchRateDeltaDPS = _sh.pitchRateDTermFilter.filter(pitchRateDPS - _sh.PIDS[PITCH_RATE_DPS].getPreviousMeasurement());
    _ahM.outputs[PITCH_RATE_DPS] = _sh.PIDS[PITCH_RATE_DPS].updateDelta(pitchRateDPS, pitchRateDeltaDPS*_rxC.TPA*_ahM.dMaxMultiplier[PITCH_RATE_DPS], deltaT);
    // filter the output
    _ahM.outputs[PITCH_RATE_DPS] = _sh.outputFilters[PITCH_RATE_DPS].filter(_ahM.outputs[PITCH_RATE_DPS]);

    // DTerm is zero for yawRate, so call updatePI() with no DTerm filtering, no TPA, and no DMax
    const float yawRateDPS = yawRateNED_DPS(gyroENU_RPS);
    _ahM.outputs[YAW_RATE_DPS] = _sh.PIDS[YAW_RATE_DPS].updatePI(yawRateDPS, deltaT);
    // filter the output
    _ahM.outputs[YAW_RATE_DPS] = _sh.outputFilters[YAW_RATE_DPS].filter(_ahM.outputs[YAW_RATE_DPS]);

    // The VehicleControllerTask is waiting on the message queue, so signal it so that there is output data available.
    // This will result in outputToMixer() being called by the scheduler
    const VehicleControllerMessageQueue::queue_item_t queueItem {
        .throttle = _sh.outputThrottle,
        .roll = _ahM.outputs[ROLL_RATE_DPS],
        .pitch = _ahM.outputs[PITCH_RATE_DPS],
        .yaw = _ahM.outputs[YAW_RATE_DPS]
    };
    SIGNAL(queueItem);
}
