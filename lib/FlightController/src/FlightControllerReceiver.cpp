#include "Debug.h"
#include "FlightController.h"
#include <RadioController.h>


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
    if (_rx.setpointTickCountCounter != 0) {
        _rx.setpointTickCountSum += tickCount;
        --_rx.setpointTickCountCounter;
        if (_rx.setpointTickCountCounter == 0) {
            _rx.setpointDeltaT = 0.001F * static_cast<float>(_rx.setpointTickCountSum) / rx_t::SETPOINT_TICKCOUNT_COUNTER_START;
            _sh.antiGravityThrottleFilter.setCutoffFrequency(_fc.antiGravityConfig.cutoff_hz, _rx.setpointDeltaT);
#if defined(USE_D_MAX)
            for (size_t ii = 0; ii < AXIS_COUNT; ++ii) {
                _sh.dMaxRangeFilter[ii].setCutoffFrequency(fc_t::D_MAX_RANGE_HZ, _rx.setpointDeltaT);
                _sh.dMaxLowpassFilter[ii].setCutoffFrequency(fc_t::D_MAX_LOWPASS_HZ, _rx.setpointDeltaT);
            }
#endif
        }
    }

    const float throttleDelta = fabsf(throttle - _rx.throttlePrevious);
    _rx.throttlePrevious = throttle;
    const float deltaT = static_cast<float>((tickCount - _rx.setpointTickCountPrevious)) * 0.001F;
    _rx.setpointTickCountPrevious = tickCount;
    float throttleDerivative = throttleDelta/deltaT;
    _debug.set(DEBUG_ANTI_GRAVITY, 0, lrintf(throttleDerivative * 100));

    const float throttleReversed = 1.0F - throttle;
    throttleDerivative *= throttleReversed * throttleReversed;
    // generally focus on the low throttle period
    if (throttle > _rx.throttlePrevious) {
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
    const float ITermAccelerator =  throttleDerivative * _fc.antiGravityIGain * ANTIGRAVITY_KI;
    _sh.PIDS[ROLL_RATE_DPS].setI(_fc.pidConstants[ROLL_RATE_DPS].ki + ITermAccelerator);
    _sh.PIDS[PITCH_RATE_DPS].setI(_fc.pidConstants[PITCH_RATE_DPS].ki + ITermAccelerator);
    _debug.set(DEBUG_ANTI_GRAVITY, 2, lrintf(1.0F + ITermAccelerator/_sh.PIDS[PITCH_RATE_DPS].getI()*1000.0F));


    // ****
    // calculate the Throttle PID Attenuation (TPA)
    // TPA is applied here to the PTerms on roll and pitch, and is used as a multiplier 
    // of the DTERM in updateOutputsUsingPIDs.
    // ****

    // _TPA is 1.0F (ie no attenuation) if throttleStick <= _TPA_Breakpoint;
    _rx.TPA = 1.0F - _fc.TPA_multiplier * std::fminf(0.0F, throttle - _fc.TPA_breakpoint);
    _debug.set(DEBUG_TPA, 0, lrintf(_rx.TPA * 1000));

    // ****
    // use TPA and anti-gravity to adjust the PTerms on roll and pitch
    // ****

    // attenuate roll if setpoint greater than 50 DPS, half at 100 DPS
    const float attenuatorRoll = std::fmaxf(fabsf(_sh.PIDS[ROLL_RATE_DPS].getSetpoint()) / 50.0F, 1.0F);
    const float PTermBoostRoll = 1.0F + (throttleDerivative *_fc.antiGravityPGain / attenuatorRoll);
    _sh.PIDS[ROLL_RATE_DPS].setP(_fc.pidConstants[ROLL_RATE_DPS].kp * PTermBoostRoll * _rx.TPA);

    // attenuate pitch if setpoint greater than 50 DPS, half at 100 DPS
    const float attenuatorPitch = std::fmaxf(fabsf(_sh.PIDS[PITCH_RATE_DPS].getSetpoint()) / 50.0F, 1.0F);
    const float PTermBoostPitch = 1.0F + (throttleDerivative *_fc.antiGravityPGain / attenuatorPitch);
    _sh.PIDS[PITCH_RATE_DPS].setP(_fc.pidConstants[PITCH_RATE_DPS].kp * PTermBoostPitch * _rx.TPA);
    _debug.set(DEBUG_ANTI_GRAVITY, 3, lrintf(PTermBoostPitch * 1000.0F));
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
void FlightController::updateSetpoints(const controls_t& controls)
{
    detectCrashOrSpin();

    setControlMode(controls.controlMode);

    // output throttle may be changed by spin recovery
    _sh.outputThrottle = controls.throttleStick;

    applyDynamicPID_AdjustmentsOnThrottleChange(controls.throttleStick, controls.tickCount);

    //!!TODO: filter the roll and stick angles
    // Pushing the ROLL stick to the right gives a positive value of rollStick and we want this to be left side up.
    // For NED left side up is positive roll, so sign of setpoint is same sign as rollStick.
    // So sign of _rollStick is left unchanged.
    _sh.PIDS[ROLL_RATE_DPS].setSetpoint(controls.rollStickDPS);
    _sh.PIDS[ROLL_ANGLE_DEGREES].setSetpoint(controls.rollStickDegrees);
    _sh.PIDS[ROLL_SIN_ANGLE].setSetpoint(sinf(controls.rollStickDegrees * degreesToRadians));

    // Pushing the  PITCH stick forward gives a positive value of _pitchStick and we want this to be nose up.
    // For NED nose up is positive pitch, so sign of setpoint is opposite sign as _pitchStick.
    // So sign of _pitchStick is negated.
    _sh.PIDS[PITCH_RATE_DPS].setSetpoint(-controls.pitchStickDPS);
    _sh.PIDS[PITCH_ANGLE_DEGREES].setSetpoint(-controls.pitchStickDegrees);
    _sh.PIDS[ROLL_SIN_ANGLE].setSetpoint(sinf(-controls.pitchStickDegrees * degreesToRadians));

    // Pushing the YAW stick to the right gives a positive value of _yawStick and we want this to be nose right.
    // For NED nose left is positive yaw, so sign of setpoint is same as sign of _yawStick.
    // So sign of _yawStick is left unchanged.
    _sh.PIDS[YAW_RATE_DPS].setSetpoint(controls.yawStickDPS);
    _rx.yawRateSetpointDPS = controls.yawStickDPS;

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
    _rx.useAngleMode = (_fc.controlMode == CONTROL_MODE_ANGLE || (_radioController.getFailsafePhase() != RadioController::FAILSAFE_IDLE)) && !_sh.groundMode;
}

/*!
NOTE: CALLED FROM WITHIN THE RECEIVER TASK

Detect crash or yaw spin. Runs in context of Receiver Task.
*/
void FlightController::detectCrashOrSpin()
{
    if (_fc.yawSpinThresholdDPS !=0.0F && fabsf(_sh.PIDS[YAW_RATE_DPS].getPreviousMeasurement()) > _fc.yawSpinThresholdDPS) {
        // yaw spin detected
        _sh.yawSpinRecovery = true;
        switchPID_integrationOff();
    }
}
