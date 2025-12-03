#include "Autopilot.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "IMU_Filters.h"

#include <Blackbox.h>
#include <Debug.h>
#include <MSP_Box.h>
#include <MotorMixerBase.h>
#include <ReceiverBase.h>

#include <cmath>

Cockpit::Cockpit(ReceiverBase& receiver, FlightController& flightController, Autopilot& autopilot, IMU_Filters& imuFilters, Debug& debug, NonVolatileStorage& nvs) :
    CockpitBase(receiver),
    _flightController(flightController),
    _autopilot(autopilot),
    _imuFilters(imuFilters),
    _debug(debug),
    _nvs(nvs)
{
    _flightController.setYawSpinThresholdDPS(1.25F*applyRates(YAW, 1.0F));
}

void Cockpit::setRebootRequired()
{
    _rebootRequired = true;
}

bool Cockpit::getRebootRequired() const
{
    return _rebootRequired;
}

bool Cockpit::isArmed() const
{
    return _armingFlags & ARMED;
}

bool Cockpit::wasEverArmed() const
{
    return _armingFlags & WAS_EVER_ARMED;
}

void Cockpit::setArmed()
{
    _armingFlags |= (ARMED | WAS_EVER_ARMED);
    _flightController.motorsSwitchOn();
}

void Cockpit::setDisarmed()
{
    _armingFlags &= ~ARMED;
    _flightController.motorsSwitchOff();
    _failsafe.phase = FAILSAFE_DISARMED;
}

void Cockpit::setArmingDisabledFlag(arming_disabled_flags_e flag)
{
    _armingDisabledFlags |= static_cast<uint32_t>(flag);
}

void Cockpit::clearArmingDisabledFlag(arming_disabled_flags_e flag)
{
    _armingDisabledFlags &= ~static_cast<uint32_t>(flag);
}

bool Cockpit::isFlightModeFlagSet(uint32_t flightModeFlag) const
{
    return _flightModeFlags & flightModeFlag;
}

void Cockpit::setFlightModeFlag(uint32_t flightModeFlag)
{
    _flightModeFlags |= flightModeFlag;
}

void Cockpit::clearFlightModeFlag(uint32_t flightModeFlag)
{
    _flightModeFlags &= ~flightModeFlag;
}


uint32_t Cockpit::getFlightModeFlags() const
{
    return _flightModeFlags;
}

bool Cockpit::isRcModeActive(uint8_t rcMode) const
{
    if (rcMode == MSP_Box::BOX_OSD) {
        return false;
    }
    if (rcMode == MSP_Box::BOX_STICK_COMMAND_DISABLE) {
        return false;
    }
    return false; // !!TODO rcMode
}

void Cockpit::setRatesToPassThrough()
{
    _rates.rcRates = { 100, 100, 100 }; // center sensitivity
    _rates.rcExpos = { 0, 0, 0}; // movement sensitivity, nonlinear
    _rates.rates   = { 0, 0, 0 }; // movement sensitivity, linear
    //_rates.ratesType = RATES_TYPE_ACTUAL;
}

inline float constrainToLimit(float value, uint16_t limit)
{
    const auto limitF = static_cast<float>(limit);
    return value < -limitF ? -limitF : value > limitF ? limitF : value;
}

float Cockpit::applyRates(size_t axis, float rcCommand) const
{
    const float rcCommand2 = rcCommand * rcCommand;
    const float rcCommandAbs = std::fabs(rcCommand);

    float expo = _rates.rcExpos[axis] / 100.0F;
    expo = rcCommandAbs*rcCommand*(expo*(rcCommand2*rcCommand2 - 1.0F) + 1.0F);

    const float centerSensitivity = _rates.rcRates[axis];
    const float stickMovement = std::fmaxf(0, _rates.rates[axis] - centerSensitivity);
    const float angleRate = 10.0F * (rcCommand*centerSensitivity + expo*stickMovement);
    //const float angleRate = 0.01F * (rcCommand*centerSensitivity + expo*stickMovement);

    return constrainToLimit(angleRate, _rates.rateLimits[axis]);
}

/*!
Map throttle in range [-1.0F, 1.0F] to a parabolic curve
in the range [-_rates.throttleLimitPercent) / 100.0F, _rates.throttleLimitPercent) / 100.0F]
*/
float Cockpit::mapThrottle(float throttle) const
{
    // alpha=0 gives a linear response, alpha=1 gives a parabolic (x^2) curve
    const float alpha = static_cast<float>(_rates.throttleExpo) / 255.0F;
    throttle *= 1.0F - alpha*(1.0F - (throttle < 0.0F ? -throttle : throttle));
    return throttle * static_cast<float>(_rates.throttleLimitPercent) / 100.0F;
}

void Cockpit::handleOnOffSwitch()
{
    if (_receiver.getSwitch(ReceiverBase::MOTOR_ON_OFF_SWITCH)) {
        _onOffSwitchPressed = true;
    } else {
        if (_onOffSwitchPressed) {
            // motorOnOff false and _onOffPressed true means the  on/off button is being released, so toggle the motor state
            if (_flightController.motorsIsOn()) {
                setDisarmed();
                if (_blackbox) {
                    _blackbox->finish();
                    _flightController.setBlackboxActive(false);
                }
            } else {
                if (_blackbox) {
                    _blackbox->start(Blackbox::start_t{
                        .debugMode = static_cast<uint16_t>(_debug.getMode()),
                        .motorCount = static_cast<uint8_t>(_flightController.getMotorMixer().getMotorCount()),
                        .servoCount = static_cast<uint8_t>(_flightController.getMotorMixer().getServoCount())
                    });
                    _flightController.setBlackboxActive(true);
                }
                setArmed();
            }
            _onOffSwitchPressed = false;
        }
    }
}

/*!
Called from Receiver Task.
*/
void Cockpit::updateControls(const controls_t& controls)
{
    // failsafe handling
    _failsafe.phase = FAILSAFE_IDLE; // we've received a packet, so exit failsafe if we were in it
    _failsafe.tickCount = controls.tickCount;

    handleOnOffSwitch();
    // if either angle mode or altitude mode is selected then use CONTROL_MODE_ANGLE
    enum { CONTROL_MODE_CHANNEL = ReceiverBase::AUX2, ALTITUDE_MODE_CHANNEL = ReceiverBase::AUX3 };
    if (_receiver.getChannelPWM(CONTROL_MODE_CHANNEL)) {
        _flightModeFlags |= ANGLE_MODE;
    } else {
        _flightModeFlags &= ~ANGLE_MODE;
    }
    if (_receiver.getChannelPWM(ALTITUDE_MODE_CHANNEL)) {
        if ((_flightModeFlags & ALT_HOLD_MODE) == 0) {
            // not currently in altitude hold mode, so set the altitude hold setpoint
            if (_autopilot.setAltitudeHoldSetpoint()) {
                // only switch to altitude hold mode if the autopilot supports it
                _flightModeFlags |= ALT_HOLD_MODE;
            }
        }
    }
    if (_flightModeFlags & (POS_HOLD_MODE | GPS_HOME_MODE | GPS_RESCUE_MODE)) {
        const FlightController::controls_t flightControls = _autopilot.calculateFlightControls(controls, _flightModeFlags);
        _flightController.updateSetpoints(flightControls);
        return;
    }

    FlightController::control_mode_e controlMode = (_flightModeFlags & ANGLE_MODE) ? FlightController::CONTROL_MODE_ANGLE : FlightController::CONTROL_MODE_RATE;

    float throttleStick {};
    if (_flightModeFlags & ALT_HOLD_MODE) {
        throttleStick = _autopilot.calculateThrottleForAltitudeHold(controls);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
    } else {
        throttleStick = mapThrottle(controls.throttleStick);
    }

    // map the radio controls to FlightController units
    const FlightController::controls_t flightControls = {
        .tickCount = controls.tickCount,
        .throttleStick = throttleStick,
        .rollStickDPS = applyRates(Cockpit::ROLL, controls.rollStick),
        .pitchStickDPS = applyRates(Cockpit::PITCH, controls.pitchStick),
        .yawStickDPS = applyRates(Cockpit::YAW, controls.yawStick),
        .rollStickDegrees = controls.rollStick * _maxRollAngleDegrees,
        .pitchStickDegrees = controls.pitchStick * _maxPitchAngleDegrees,
        .controlMode = controlMode
    };

    _flightController.updateSetpoints(flightControls);
}

void Cockpit::checkFailsafe(uint32_t tickCount)
{
    _flightController.detectCrashOrSpin();

    if ((tickCount - _failsafe.tickCount > _failsafe.tickCountThreshold) && (_failsafe.phase != FAILSAFE_DISARMED)) {
        // We've had tickCountThreshold ticks without a packet, so we seem to have lost contact with the transmitter,
        if ((tickCount - _failsafe.tickCount < _failsafe.tickCountSwitchOffThreshold)) {
            // failsafe detected, so zero all sticks, set throttle to its failsafe value, and switch to angle mode
            _failsafe.phase = FAILSAFE_RX_LOSS_DETECTED;
            const FlightController::controls_t flightControls = {
                .tickCount = tickCount,
                .throttleStick = (static_cast<float>(_failsafeConfig.throttle_pwm) - ReceiverBase::CHANNEL_LOW_F) / ReceiverBase::CHANNEL_RANGE_F,
                .rollStickDPS = 0.0F,
                .pitchStickDPS = 0.0F,
                .yawStickDPS = 0.0F,
                .rollStickDegrees = 0.0F,
                .pitchStickDegrees = 0.0F,
                .controlMode = FlightController::CONTROL_MODE_ANGLE
            };
            _flightController.updateSetpoints(flightControls);
        } else {
            // we've lost contact for an extended period, so disarm.
            _failsafe.phase = FAILSAFE_DISARMED;
            setDisarmed();
        }
    }
}

void Cockpit::setFailsafeConfig(const failsafe_config_t& failsafeConfig)
{
    _failsafeConfig = failsafeConfig;
}

void Cockpit::setRX_Config(const rx_config_t& rxConfig)
{
    _rxConfig = rxConfig;
}
