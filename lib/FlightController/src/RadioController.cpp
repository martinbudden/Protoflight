#include "FlightController.h"
#include "RadioController.h"
#include <cmath>

static const RadioController::rates_t defaultRates {
    .rateLimits = { RadioController::RATE_LIMIT_MAX, RadioController::RATE_LIMIT_MAX, RadioController::RATE_LIMIT_MAX},
    .rcRates = { 7, 7, 7 },
    .rcExpos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttleMidpoint = 50,
    .throttleExpo = 0,
    .throttleLimitType = RadioController::THROTTLE_LIMIT_TYPE_OFF,
    .throttleLimitPercent = 100,
    .ratesType = RadioController::RATES_TYPE_ACTUAL
};

RadioController::RadioController(ReceiverBase& receiver) :
    RadioControllerBase(receiver),
    _rates(defaultRates)
{
}

void RadioController::setRatesToPassThrough()
{
    _rates.rcRates = { 100, 100, 100 }; // center sensitivity
    _rates.rcExpos = { 0, 0, 0}; // movement sensitivity, nonlinear
    _rates.rates   = { 0, 0, 0 }; // movement sensitivity, linear
    _rates.ratesType = RATES_TYPE_ACTUAL;
}

inline float constrain(float value, int16_t limit)
{
    const auto limitF = static_cast<float>(limit);
    return value < -limitF ? -limitF : value > limitF ? limitF : value;
}

float RadioController::applyRates(size_t axis, float rcCommand) const
{
    const float rcCommand2 = rcCommand * rcCommand;
    const float rcCommandAbs = fabsf(rcCommand);

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

void RadioController::updateControls(const controls_t& controls)
{
    // failsafe handling
    _receiverInUse = true;
    _failsafePhase = FAILSAFE_IDLE; // we've received a packet, so exit failsafe if we were in it

    // handle the on/off switch
    if (_receiver.getSwitch(ReceiverBase::MOTOR_ON_OFF_SWITCH)) {
        _onOffSwitchPressed = true;
    } else {
        if (_onOffSwitchPressed) {
            // motorOnOff false and _onOffPressed true means the  on/off button is being released, so toggle the motor state
            _flightController->motorsToggleOnOff();
            _onOffSwitchPressed = false;
        }
    }

    // map the radio controls to FlightController units
    const FlightController::controls_t flightControls = {
        .tickCount = controls.tickCount,
        .throttleStick = mapThrottle(controls.throttleStick),
        .rollStickDPS = applyRates(RadioController::ROLL, controls.rollStick),
        .pitchStickDPS = applyRates(RadioController::PITCH, controls.pitchStick),
        .yawStickDPS = applyRates(RadioController::YAW, controls.yawStick),
        .rollStickDegrees = controls.rollStick * _maxRollAngleDegrees,
        .pitchStickDegrees = controls.pitchStick * _maxPitchAngleDegrees,
        .controlMode =
            _receiver.getSwitch(1) ? FlightController::CONTROL_MODE_ALTITUDE_HOLD :
            _receiver.getSwitch(0) ? FlightController::CONTROL_MODE_ANGLE : FlightController::CONTROL_MODE_RATE
    };

    _flightController->updateSetpoints(flightControls);
}

uint32_t RadioController::getFailsafePhase() const
{
    return _failsafePhase;
}

void RadioController::setFailsafe(const failsafe_t& failsafe)
{
    _failsafe = failsafe;
}

void RadioController::checkFailsafe(uint32_t tickCount)
{
    _flightController->detectCrashOrSpin(tickCount);

    if ((tickCount - _failsafeTickCount > _failsafeTickCountThreshold) && _receiverInUse) {
        // _receiverInUse is initialized to false, so the motors won't turn off it the transmitter hasn't been turned on yet.
        // We've had 1500 ticks (1.5 seconds) without a packet, so we seem to have lost contact with the transmitter,
        // so enter failsafe mode.
        _failsafePhase = FAILSAFE_RX_LOSS_DETECTED;
        if ((tickCount - _failsafeTickCount > _failsafeTickCountSwitchOffThreshold)) {
            _flightController->motorsSwitchOff();
            _receiverInUse = false; // set to false to allow us to switch the motors on again if we regain a signal
        }
    }
}
