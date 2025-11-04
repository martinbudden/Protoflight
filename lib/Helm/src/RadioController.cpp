#include "Autopilot.h"
#include "FlightController.h"
#include "RadioController.h"

#include <Blackbox.h>
#include <Debug.h>
#include <MotorMixerBase.h>
#include <ReceiverBase.h>
#include <cmath>


RadioController::RadioController(ReceiverBase& receiver, FlightController& flightController, Autopilot& autopilot, Debug& debug, const rates_t& rates) :
    RadioControllerBase(receiver),
    _flightController(flightController),
    _autopilot(autopilot),
    _debug(debug),
    _rates(rates)
{
    _flightController.setYawSpinThresholdDPS(1.25F*applyRates(YAW, 1.0F));
}

void RadioController::setRatesToPassThrough()
{
    _rates.rcRates = { 100, 100, 100 }; // center sensitivity
    _rates.rcExpos = { 0, 0, 0}; // movement sensitivity, nonlinear
    _rates.rates   = { 0, 0, 0 }; // movement sensitivity, linear
    //_rates.ratesType = RATES_TYPE_ACTUAL;
}

inline float constrain(float value, uint16_t limit)
{
    const auto limitF = static_cast<float>(limit);
    return value < -limitF ? -limitF : value > limitF ? limitF : value;
}

float RadioController::applyRates(size_t axis, float rcCommand) const
{
    const float rcCommand2 = rcCommand * rcCommand;
    const float rcCommandAbs = std::fabs(rcCommand);

    float expo = _rates.rcExpos[axis] / 100.0F;
    expo = rcCommandAbs*rcCommand*(expo*(rcCommand2*rcCommand2 - 1.0F) + 1.0F);

    const float centerSensitivity = _rates.rcRates[axis];
    const float stickMovement = std::fmaxf(0, _rates.rates[axis] - centerSensitivity);
    const float angleRate = 10.0F * (rcCommand*centerSensitivity + expo*stickMovement);
    //const float angleRate = 0.01F * (rcCommand*centerSensitivity + expo*stickMovement);

    return constrain(angleRate, _rates.rateLimits[axis]);
}

/*!
Map throttle in range [-1.0F, 1.0F] to a parabolic curve
in the range [-_rates.throttleLimitPercent) / 100.0F, _rates.throttleLimitPercent) / 100.0F]
*/
float RadioController::mapThrottle(float throttle) const
{
    // alpha=0 gives a linear response, alpha=1 gives a parabolic (x^2) curve
    const float alpha = static_cast<float>(_rates.throttleExpo) / 255.0F;
    throttle *= 1.0F - alpha*(1.0F - (throttle < 0.0F ? -throttle : throttle));
    return throttle * static_cast<float>(_rates.throttleLimitPercent) / 100.0F;
}

void RadioController::handleOnOffSwitch()
{
    if (_receiver.getSwitch(ReceiverBase::MOTOR_ON_OFF_SWITCH)) {
        _onOffSwitchPressed = true;
    } else {
        if (_onOffSwitchPressed) {
            // motorOnOff false and _onOffPressed true means the  on/off button is being released, so toggle the motor state
            if (_flightController.motorsIsOn()) {
                _flightController.motorsSwitchOff();
                if (_blackbox) {
                    _blackbox->finish();
                    _flightController.setBlackboxActive(false);
                }
            } else {
                if (_blackbox) {
                    _blackbox->start(Blackbox::start_t{
                        .debugMode = static_cast<uint16_t>(_debug.getMode()),
                        .motorCount = static_cast<uint8_t>(_flightController.getMixer().getMotorCount()),
                        .servoCount = static_cast<uint8_t>(_flightController.getMixer().getServoCount())
                    });
                    _flightController.setBlackboxActive(true);
                }
                _flightController.motorsSwitchOn();
            }
            _onOffSwitchPressed = false;
        }
    }
}

/*!
Called from Receiver Task.
*/
void RadioController::updateControls(const controls_t& controls)
{
    // failsafe handling
    _receiverInUse = true;
    _failsafePhase = FAILSAFE_IDLE; // we've received a packet, so exit failsafe if we were in it
    _failsafeTickCount = controls.tickCount;

    handleOnOffSwitch();
    // if either angle mode or altitude mode is selected then use CONTROL_MODE_ANGLE
    enum { CONTROL_MODE_CHANNEL = ReceiverBase::AUX2, ALTITUDE_MODE_CHANNEL = ReceiverBase::AUX3 };
    if (_receiver.getChannelRaw(CONTROL_MODE_CHANNEL)) {
        _flightMode |= ANGLE_MODE;
    } else {
        _flightMode &= ~ANGLE_MODE;
    }
    if (_receiver.getChannelRaw(ALTITUDE_MODE_CHANNEL)) {
        if ((_flightMode & ALTITUDE_HOLD_MODE) == 0) {
            // not currently in altitude hold mode, so set the altitude hold setpoint
            if (_autopilot.setAltitudeHoldSetpoint()) {
                // only switch to altitude hold mode if the autopilot supports it
                _flightMode |= ALTITUDE_HOLD_MODE;
            }
        }
    }
    if (_flightMode & (POSITION_HOLD_MODE | RETURN_TO_HOME_MODE | WAYPOINT_MODE)) {
        const FlightController::controls_t flightControls = _autopilot.calculateFlightControls(controls, _flightMode);
        _flightController.updateSetpoints(flightControls);
        return;
    }

    FlightController::control_mode_e controlMode = (_flightMode & ANGLE_MODE) ? FlightController::CONTROL_MODE_ANGLE : FlightController::CONTROL_MODE_RATE;

    float throttleStick {};
    if (_flightMode & ALTITUDE_HOLD_MODE) {
        throttleStick = _autopilot.calculateThrottleForAltitudeHold(controls);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
    } else {
        throttleStick = mapThrottle(controls.throttleStick);
    }

    // map the radio controls to FlightController units
    const FlightController::controls_t flightControls = {
        .tickCount = controls.tickCount,
        .throttleStick = throttleStick,
        .rollStickDPS = applyRates(RadioController::ROLL, controls.rollStick),
        .pitchStickDPS = applyRates(RadioController::PITCH, controls.pitchStick),
        .yawStickDPS = applyRates(RadioController::YAW, controls.yawStick),
        .rollStickDegrees = controls.rollStick * _maxRollAngleDegrees,
        .pitchStickDegrees = controls.pitchStick * _maxPitchAngleDegrees,
        .controlMode = controlMode
    };

    _flightController.updateSetpoints(flightControls);
}

void RadioController::setFailsafe(const failsafe_t& failsafe)
{
    _failsafe = failsafe;
}

void RadioController::checkFailsafe(uint32_t tickCount)
{
    _flightController.detectCrashOrSpin();

    if ((tickCount - _failsafeTickCount > _failsafeTickCountThreshold) && _receiverInUse) {
        // _receiverInUse is initialized to false, so the motors won't turn off it the transmitter hasn't been turned on yet.
        // We've had 1500 ticks (1.5 seconds) without a packet, so we seem to have lost contact with the transmitter,
        // so enter failsafe mode.
        _failsafePhase = FAILSAFE_RX_LOSS_DETECTED;
        if ((tickCount - _failsafeTickCount > _failsafeTickCountSwitchOffThreshold)) {
            _flightController.motorsSwitchOff();
            _receiverInUse = false; // set to false to allow us to switch the motors on again if we regain a signal
        } else {
            // failsafe detected, so zero all sticks and set throttle to 25%
            const FlightController::controls_t flightControls = {
                .tickCount = tickCount,
                .throttleStick = 0.25F,
                .rollStickDPS = 0.0F,
                .pitchStickDPS = 0.0F,
                .yawStickDPS = 0.0F,
                .rollStickDegrees = 0.0F,
                .pitchStickDegrees = 0.0F,
                .controlMode = FlightController::CONTROL_MODE_ANGLE
            };
            _flightController.updateSetpoints(flightControls);
        }
    }
}
