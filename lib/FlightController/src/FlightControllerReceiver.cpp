#include "DynamicNotchFilter.h"
#include "FlightController.h"
#include <Debug.h>

// NOLINTBEGIN(cppcoreguidelines-macro-usage,bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
// #defines to catch inadvertent use of _fcM or _ahM in this file.
#define _fcM "error not modifiable in this task"
#define _ahM "error not modifiable in this task"
// NOLINTEND(cppcoreguidelines-macro-usage,bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)


/*!
NOTE: CALLED FROM INITIALIZATION
*/
void FlightController::setYawSpinThresholdDPS(float yawSpinThresholdDPS)
{
#if defined(USE_YAW_SPIN)
    _sh.yawSpinThresholdDPS = yawSpinThresholdDPS;
#else
    (void)yawSpinThresholdDPS;
#endif
}

void FlightController::initializeSetpointFilters(float setpointDeltaT) // NOLINT(readability-make-member-function-const)
{
    if (_antiGravityConfig.cutoff_hz == 0) {
        _sh.antiGravityThrottleFilter.setToPassthrough();
    } else {
        _sh.antiGravityThrottleFilter.setCutoffFrequency(_antiGravityConfig.cutoff_hz, setpointDeltaT);
    }
    // Feedforward filters
    if (_filtersConfig.rc_smoothing_feedforward_cutoff == 0) {
        for (auto& filter : _sh.setpointDerivativeFilters) {
            filter.setToPassthrough();
        }
    } else {
        for (auto& filter : _sh.setpointDerivativeFilters) {
            filter.setCutoffFrequencyAndReset(_filtersConfig.rc_smoothing_feedforward_cutoff, setpointDeltaT);
        }
    }

#if defined(USE_D_MAX)
    for (auto& filter : _sh.dMaxRangeFilters) {
        filter.setCutoffFrequency(DMAX_RANGE_HZ, setpointDeltaT);
    }
    for (auto& filter : _sh.dMaxLowpassFilters) {
        filter.setCutoffFrequency(DMAX_LOWPASS_HZ, setpointDeltaT);
    }
#endif
#if defined(USE_ITERM_RELAX)
    for (auto& filter : _sh.iTermRelaxFilters) {
        filter.setCutoffFrequency(_iTermRelaxConfig.iterm_relax_cutoff, setpointDeltaT);
    }
#endif
}

/*!
NOTE: CALLED FROM WITHIN THE RECEIVER TASK

Dynamic PID adjustments made when throttle changes:

These include:

Throttle PID Attenuation: lowers the roll rate and pitch rate P and D terms when the throttle is high

Anti-gravity: adjusts the roll rate and pitch rate P and I terms when the throttle is moving quickly

D_MAX is not applied here, since it depends on the gyro value and so needs to be calculated in updateOutputsUsingPIDs()
*/
void FlightController::applyDynamicPID_AdjustmentsOnThrottleChange(float throttle, uint32_t tickCount)
{
    // We don't know the period at which setpoints are updated (it depends on the receiver) so calculate this
    // so we can set the filters' cutoff frequency
    if (_rxM.setpointTickCountCounter != 0) {
        _rxM.setpointTickCountSum += tickCount;
        --_rxM.setpointTickCountCounter;
        if (_rxM.setpointTickCountCounter == 0) {
            _rxM.setpointDeltaT = 0.001F * static_cast<float>(_rxM.setpointTickCountSum) / static_cast<float>(rx_t::SETPOINT_TICKCOUNT_COUNTER_START);
            initializeSetpointFilters(_rxM.setpointDeltaT);
        }
    }

    const float throttleDelta = std::fabs(throttle - _rxM.throttlePrevious);
    _rxM.throttlePrevious = throttle;
    const float deltaT = static_cast<float>((tickCount - _rxM.setpointTickCountPrevious)) * 0.001F;
    _rxM.setpointTickCountPrevious = tickCount;
    float throttleDerivative = throttleDelta/deltaT;
    _debug.set(DEBUG_ANTI_GRAVITY, 0, lrintf(throttleDerivative * 100));

    const float throttleReversed = 1.0F - throttle;
    throttleDerivative *= throttleReversed * throttleReversed;
    // generally focus on the low throttle period
    if (throttle > _rxM.throttlePrevious) {
        throttleDerivative *= throttleReversed * 0.5F;
        // when increasing throttle, focus even more on the low throttle range
    }
    // filtering suppresses peaks relative to troughs and prolongs the anti-gravity effects
    throttleDerivative = _sh.antiGravityThrottleFilter.filter(throttleDerivative);
    _debug.set(DEBUG_ANTI_GRAVITY, 1, lrintf(throttleDerivative * 100));

    // ****
    // use anti-gravity to adjust the ITerms on roll and pitch
    // ****

    static constexpr float ANTIGRAVITY_KI = 0.34F;
    const float iTermAccelerator =  throttleDerivative * _antiGravity.IGain * ANTIGRAVITY_KI;
    _sh.PIDS[ROLL_RATE_DPS].setI(_fcC.pidConstants[ROLL_RATE_DPS].ki + iTermAccelerator);
    _sh.PIDS[PITCH_RATE_DPS].setI(_fcC.pidConstants[PITCH_RATE_DPS].ki + iTermAccelerator);
    _debug.set(DEBUG_ANTI_GRAVITY, 2, lrintf(1.0F + iTermAccelerator/_sh.PIDS[PITCH_RATE_DPS].getI()*1000.0F));


    // ****
    // calculate the Throttle PID Attenuation (TPA)
    // TPA is applied here to the PTerms on roll and pitch, and is used as a multiplier
    // of the DTERM in updateOutputsUsingPIDs.
    // ****

    // _TPA is 1.0F (ie no attenuation) if throttleStick <= _tpaBreakpoint;
    _rxM.TPA = 1.0F - _tpa.multiplier * std::fminf(0.0F, throttle - _tpa.breakpoint);
    _debug.set(DEBUG_TPA, 0, lrintf(_rxM.TPA * 1000));

    // ****
    // use TPA and anti-gravity to adjust the PTerms on roll and pitch
    // ****

    // attenuate roll if setpoint greater than 50 DPS, half at 100 DPS
    const float attenuatorRoll = std::fmaxf(std::fabs(_sh.PIDS[ROLL_RATE_DPS].getSetpoint()) / 50.0F, 1.0F);
    const float PTermBoostRoll = 1.0F + (throttleDerivative *_antiGravity.PGain / attenuatorRoll);
    _sh.PIDS[ROLL_RATE_DPS].setP(_fcC.pidConstants[ROLL_RATE_DPS].kp * PTermBoostRoll * _rxM.TPA);

    // attenuate pitch if setpoint greater than 50 DPS, half at 100 DPS
    const float attenuatorPitch = std::fmaxf(std::fabs(_sh.PIDS[PITCH_RATE_DPS].getSetpoint()) / 50.0F, 1.0F);
    const float PTermBoostPitch = 1.0F + (throttleDerivative *_antiGravity.PGain / attenuatorPitch);
    _sh.PIDS[PITCH_RATE_DPS].setP(_fcC.pidConstants[PITCH_RATE_DPS].kp * PTermBoostPitch * _rxM.TPA);
    _debug.set(DEBUG_ANTI_GRAVITY, 3, lrintf(PTermBoostPitch * 1000.0F));
}

void FlightController::clearDynamicPID_Adjustments()
{
    _sh.PIDS[ROLL_RATE_DPS].setI(_fcC.pidConstants[ROLL_RATE_DPS].ki);
    _sh.PIDS[PITCH_RATE_DPS].setI(_fcC.pidConstants[PITCH_RATE_DPS].ki);
    _rxM.TPA = 1.0F;
    _sh.PIDS[ROLL_RATE_DPS].setP(_fcC.pidConstants[ROLL_RATE_DPS].kp);
    _sh.PIDS[PITCH_RATE_DPS].setP(_fcC.pidConstants[PITCH_RATE_DPS].kp);
}

/*!
NOTE: CALLED FROM WITHIN THE RECEIVER TASK

Use the new joystick values from the receiver to update the PID setpoints
using the NED (North-East-Down) coordinate convention.

NOTE: this function is called from `updateControls()` in the ReceiverTask,
as a result of receiving new values from the receiver.
How often it is called depends on the type of transmitter and receiver,
but is typically at intervals of between 40 milliseconds and 5 milliseconds (ie 25Hz to 200Hz).
In particular it runs much less frequently than `updateOutputsUsingPIDs()` which typically runs at 1000Hz to 8000Hz.
*/
void FlightController::updateSetpoints(const controls_t& controls, failsafe_e failsafe)
{
    detectCrashOrSpin();

    setControlMode(controls.controlMode);
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    if (_dynamicNotchFilter) {
        _dynamicNotchFilter->setThrottle(controls.throttleStick);
    }
#endif

    // output throttle may be changed by spin recovery
    _sh.outputThrottle = controls.throttleStick;

    if (failsafe == FAILSAFE_ON || _sh.crashDetected || _sh.yawSpinRecovery) {
        clearDynamicPID_Adjustments();
    } else {
        applyDynamicPID_AdjustmentsOnThrottleChange(controls.throttleStick, controls.tickCount);
    }

    //
    // Roll axis
    //
    // Pushing the ROLL stick to the right gives a positive value of rollStick and we want this to be left side up.
    // For NED left side up is positive roll, so sign of setpoint is same sign as rollStick.
    // So sign of _rollStick is left unchanged.
    if (!_rxM.useAngleMode) {
        _sh.PIDS[ROLL_RATE_DPS].setSetpoint(controls.rollStickDPS);
    }
    if (_rxM.setpointDeltaT != 0) {
        if (failsafe == FAILSAFE_ON || _sh.crashDetected || _sh.yawSpinRecovery) {
            _sh.PIDS[ROLL_RATE_DPS].setSetpointDerivative(0.0F);
        } else {
            float setpointDerivative = _sh.PIDS[ROLL_RATE_DPS].getSetpointDelta() / _rxM.setpointDeltaT;
            setpointDerivative = _sh.setpointDerivativeFilters[ROLL_RATE_DPS].filter(setpointDerivative);
            _sh.PIDS[ROLL_RATE_DPS].setSetpointDerivative(setpointDerivative);
        }
    }
#if defined(USE_ITERM_RELAX)
    _rxM.setpointLPs[ROLL_RATE_DPS] = _sh.iTermRelaxFilters[ROLL_RATE_DPS].filter(controls.rollStickDPS);
    _rxM.setpointHPs[ROLL_RATE_DPS] = std::fabs(controls.rollStickDPS - _rxM.setpointLPs[ROLL_RATE_DPS]);
#endif
    _sh.PIDS[ROLL_ANGLE_DEGREES].setSetpoint(controls.rollStickDegrees);
#if defined(USE_SIN_ANGLE_PIDS)
    _sh.PIDS[ROLL_SIN_ANGLE].setSetpoint(sinf(controls.rollStickDegrees * DEGREES_TO_RADIANS));
#endif

    //
    // Pitch axis
    //
    // Pushing the  PITCH stick forward gives a positive value of _pitchStick and we want this to be nose down.
    // For NED nose down is negative pitch, so sign of setpoint is opposite sign as _pitchStick.
    // So sign of _pitchStick is negated.
    if (!_rxM.useAngleMode) {
        _sh.PIDS[PITCH_RATE_DPS].setSetpoint(-controls.pitchStickDPS);
    }
    if (_rxM.setpointDeltaT != 0) {
        if (failsafe == FAILSAFE_ON || _sh.crashDetected || _sh.yawSpinRecovery) {
            _sh.PIDS[PITCH_RATE_DPS].setSetpointDerivative(0.0F);
        } else {
            float setpointDerivative = _sh.PIDS[PITCH_RATE_DPS].getSetpointDelta() / _rxM.setpointDeltaT;
            setpointDerivative = _sh.setpointDerivativeFilters[PITCH_RATE_DPS].filter(setpointDerivative);
            _sh.PIDS[PITCH_RATE_DPS].setSetpointDerivative(setpointDerivative);
        }
    }
#if defined(USE_ITERM_RELAX)
    _rxM.setpointLPs[PITCH_RATE_DPS] = _sh.iTermRelaxFilters[PITCH_RATE_DPS].filter(controls.pitchStickDPS);
    _rxM.setpointHPs[PITCH_RATE_DPS] = std::fabs(controls.pitchStickDPS - _rxM.setpointLPs[PITCH_RATE_DPS]);
#endif
    _sh.PIDS[PITCH_ANGLE_DEGREES].setSetpoint(-controls.pitchStickDegrees);
#if defined(USE_SIN_ANGLE_PIDS)
    _sh.PIDS[ROLL_SIN_ANGLE].setSetpoint(sinf(-controls.pitchStickDegrees * DEGREES_TO_RADIANS));
#endif

    //
    // Yaw axis
    //
    // Pushing the YAW stick to the right gives a positive value of _yawStick and we want this to be nose right.
    // For NED nose left is positive yaw, so sign of setpoint is same as sign of _yawStick.
    // So sign of _yawStick is left unchanged.
    _sh.PIDS[YAW_RATE_DPS].setSetpoint(controls.yawStickDPS);
    _rxM.yawRateSetpointDPS = controls.yawStickDPS;

    //
    // Modes
    //
    // When in ground mode, the PID I-terms are set to zero to avoid integral windup on the ground
    if (_sh.groundMode) {
        // exit ground mode if the throttle has been above _takeOffThrottleThreshold for _takeOffTickThreshold ticks
        if (_sh.outputThrottle < _takeOffThrottleThreshold) {
            _sh.takeOffCountStart = 0;
        } else {
            const uint32_t tickCount = controls.tickCount;
            if (_sh.takeOffCountStart == 0) {
                _sh.takeOffCountStart = tickCount;
            }
            if (tickCount - _sh.takeOffCountStart > _takeOffTickThreshold) {
                _sh.groundMode = false;
                // we've exited ground mode, so we can turn on PID integration
                switchPID_integrationOn();
            }
        }
    }
    // Angle Mode is used if the controlMode is set to angle mode, or failsafe is on.
    // Angle Mode is prevented when in Ground Mode, so the aircraft doesn't try and self-level while it is still on the ground.
    // This value is cached here, to avoid evaluating a reasonably complex condition in updateOutputsUsingPIDs()
    _rxM.useAngleMode = (_fcC.controlMode >= CONTROL_MODE_ANGLE) && !_sh.groundMode;
}

/*!
NOTE: CALLED FROM WITHIN THE RECEIVER TASK

Detect crash or yaw spin. Runs in context of Receiver Task.
*/
void FlightController::detectCrashOrSpin()
{
#if defined(USE_YAW_SPIN_RECOVERY)
    if (_sh.yawSpinThresholdDPS !=0.0F && std::fabs(_sh.PIDS[YAW_RATE_DPS].getPreviousMeasurement()) > _sh.yawSpinThresholdDPS) {
        // yaw spin detected
        _sh.yawSpinRecovery = true;
        switchPID_integrationOff();
    }
#endif
#if defined(USE_CRASH_RECOVERY)
    const size_t axis = YAW_RATE_DPS;
    const PIDF pid = _sh.PIDS[axis];
    if (std::fabs(pid.getErrorRawD()) > _crash.DtermThresholdDPSPS
        && std::fabs(pid.getErrorRawP()) > _crash.gyroThresholdDPS
        && std::fabs(pid.getSetpoint()) < _crash.setpointThresholdDPS) {
        _sh.crashDetected = true;
        switchPID_integrationOff();
    }
#endif
}
