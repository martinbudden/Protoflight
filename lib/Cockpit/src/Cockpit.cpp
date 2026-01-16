#include "Autopilot.h"
#include "Cockpit.h"
#include "FlightController.h"

#include <Blackbox.h>
#include <Debug.h>
#include <MSP_Box.h>
#include <MotorMixerBase.h>
#include <ReceiverBase.h>

#include <cmath>

Cockpit::Cockpit(ReceiverBase& receiver, FlightController& flightController, Autopilot& autopilot, IMU_Filters& imuFilters, Debug& debug, NonVolatileStorage& nvs, [[maybe_unused]] const RC_Adjustments::adjustment_configs_t* defaultAdjustmentConfigs) :
    CockpitBase(receiver),
    _flightController(flightController),
    _autopilot(autopilot),
    _imuFilters(imuFilters),
    _debug(debug),
    _nvs(nvs)
#if defined(USE_RC_ADJUSTMENTS)
    ,_rcAdjustments(defaultAdjustmentConfigs)
#endif
{
    _flightController.setYawSpinThresholdDPS(1.25F*applyRates(rates_t::YAW, 1.0F));

    _mspBox.setActiveBoxId(MSP_Box::BOX_ARM);
    _mspBox.setActiveBoxId(MSP_Box::BOX_PREARM);
    _mspBox.setActiveBoxId(MSP_Box::BOX_AIRMODE);

    _mspBox.setActiveBoxId(MSP_Box::BOX_ANTIGRAVITY);

    _mspBox.setActiveBoxId(MSP_Box::BOX_ANGLE);
    _mspBox.setActiveBoxId(MSP_Box::BOX_HORIZON);
    _mspBox.setActiveBoxId(MSP_Box::BOX_ALTITUDE_HOLD);
    _mspBox.setActiveBoxId(MSP_Box::BOX_HEADFREE);
    _mspBox.setActiveBoxId(MSP_Box::BOX_HEADADJ);
    _mspBox.setActiveBoxId(MSP_Box::BOX_FPV_ANGLE_MIX);
    if (featureIsEnabled(Features::FEATURE_INFLIGHT_ACC_CALIBRATE)) {
        _mspBox.setActiveBoxId(MSP_Box::BOX_CALIBRATE);
    }
    _mspBox.setActiveBoxId(MSP_Box::BOX_ACRO_TRAINER);

    _mspBox.setActiveBoxId(MSP_Box::BOX_FAILSAFE);

    _mspBox.setActiveBoxId(MSP_Box::BOX_BEEPER_ON);
    _mspBox.setActiveBoxId(MSP_Box::BOX_BEEPER_MUTE);

    _mspBox.setActiveBoxId(MSP_Box::BOX_PARALYZE);
    _mspBox.setActiveBoxId(MSP_Box::BOX_MSP_OVERRIDE);

    _mspBox.setActiveBoxId(MSP_Box::BOX_STICK_COMMAND_DISABLE);
    _mspBox.setActiveBoxId(MSP_Box::BOX_READY);

    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_ARM]           == 0); // not used
    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_ANGLE]         == LOG2_ANGLE_MODE);
    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_HORIZON]       == LOG2_HORIZON_MODE);
    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_MAG]           == LOG2_MAG_MODE);
    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_ALTITUDE_HOLD] == LOG2_ALTITUDE_HOLD_MODE);
    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_POSITION_HOLD] == LOG2_POSITION_HOLD_MODE);
    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_HEADFREE]      == LOG2_HEADFREE_MODE);
    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_CHIRP]         == LOG2_CHIRP_MODE);
    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_PASSTHRU]      == LOG2_PASSTHRU_MODE);
    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_FAILSAFE]      == LOG2_FAILSAFE_MODE);
    static_assert(BoxIdToFlightModeMap[MSP_Box::BOX_GPS_RESCUE]    == LOG2_GPS_RESCUE_MODE);
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

uint32_t Cockpit::getFlightModeFlags() const
{
    return _flightModeFlags.to_ulong();
}

void Cockpit::setRatesToPassThrough()
{
    _rates.rcRates = { 100, 100, 100 }; // center sensitivity
    _rates.rcExpos = { 0, 0, 0}; // movement sensitivity, nonlinear
    _rates.rates   = { 0, 0, 0 }; // movement sensitivity, linear
    //_rates.ratesType = rates_t::RATES_TYPE_ACTUAL;
}

inline float clampToLimit(float value, uint16_t limit)
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

    return clampToLimit(angleRate, _rates.rateLimits[axis]);
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

void Cockpit::startBlackboxRecording()
{
    if (_blackbox) {
        _blackbox->start(Blackbox::start_t{
            .debugMode = static_cast<uint16_t>(_debug.getMode()),
            .motorCount = static_cast<uint8_t>(_flightController.getMotorMixer().getMotorCount()),
            .servoCount = static_cast<uint8_t>(_flightController.getMotorMixer().getServoCount())
        });
        _flightController.setBlackboxActive(true);
    }
}

void Cockpit::stopBlackboxRecording()
{
    if (_blackbox) {
        _blackbox->finish();
        _flightController.setBlackboxActive(false);
    }
}

void Cockpit::handleArmingSwitch()
{
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
    if (_receiver.getSwitch(ReceiverBase::MOTOR_ON_OFF_SWITCH)) {
        _onOffSwitchPressed = true;
    } else {
        if (_onOffSwitchPressed) {
            // MOTOR_ON_OFF_SWITCH is false and _onOffSwitchPressed true means that the switch was previously pressed and is now being released
            _onOffSwitchPressed = false;
            // toggle arming when the onOff switch is released
            if (isArmed()) {
                setDisarmed();
                stopBlackboxRecording();
            } else {
                if (_recordToBlackboxWhenArmed) {
                    startBlackboxRecording();
                }
                setArmed();
            }
        }
    }
#else
    if (_rcModes.isModeActive(MSP_Box::BOX_ARM)) {
        if (!isArmed()) {
            if (_recordToBlackboxWhenArmed) {
                startBlackboxRecording();
            }
            setArmed();
        }
    } else {
        if (isArmed()) {
            setDisarmed();
            stopBlackboxRecording();
        }
    }
#endif
}

/*!
Called from Receiver Task.
*/
void Cockpit::updateControls(const controls_t& controls)
{
    // failsafe handling
    _failsafe.phase = FAILSAFE_IDLE; // we've received a packet, so exit failsafe if we were in it
    _failsafe.tickCount = controls.tickCount;

    // set the RC modes according to the receiver channel values
    _rcModes.updateActivatedModes(_receiver);

    handleArmingSwitch();

#if defined(USE_RC_ADJUSTMENTS)
    // process any in-flight adjustments
    if (!_cliMode && !(_rcModes.isModeActive(MSP_Box::BOX_PARALYZE) && !isArmed())) {
        _rcAdjustments.processAdjustments(_receiver, _flightController, *this, _osd, true); //!!TODO: check true parameter
    }
#endif

    _flightModeFlags.reset();
    FlightController::control_mode_e controlMode = FlightController::CONTROL_MODE_RATE;
    if (_rcModes.isModeActive(MSP_Box::BOX_ANGLE)) {
        _flightModeFlags.set(ANGLE_MODE);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
    }
    if (_rcModes.isModeActive(MSP_Box::BOX_HORIZON)) {
        _flightModeFlags.set(HORIZON_MODE);
        // we don't support horizon mode, instead we use the horizon mode setting to invoke level race mode
        controlMode = FlightController::CONTROL_MODE_LEVEL_RACE;
    }
    if (_rcModes.isModeActive(MSP_Box::BOX_ALTITUDE_HOLD)) {
        _flightModeFlags.set(ALTITUDE_HOLD_MODE);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
        // not currently in altitude hold mode, so set the altitude hold setpoint
        if (!_autopilot.isAltitudeHoldSetpointSet()) {
            _autopilot.setAltitudeHoldSetpoint();
        }
    }
    if (_rcModes.isModeActive(MSP_Box::BOX_POSITION_HOLD)) {
        _flightModeFlags.set(POSITION_HOLD_MODE);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
    }
    if (_rcModes.isModeActive(MSP_Box::BOX_MAG)) {
        _flightModeFlags.set(MAG_MODE);
    }
    if (_rcModes.isModeActive(MSP_Box::BOX_HEADFREE)) {
        _flightModeFlags.set(HEADFREE_MODE);
    }
    if (_rcModes.isModeActive(MSP_Box::BOX_CHIRP)) {
        _flightModeFlags.set(CHIRP_MODE);
    }
    if (_rcModes.isModeActive(MSP_Box::BOX_PASSTHRU)) {
        _flightModeFlags.set(PASSTHRU_MODE);
    }
    if (_rcModes.isModeActive(MSP_Box::BOX_FAILSAFE)) {
        _flightModeFlags.set(FAILSAFE_MODE);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
    }
    if (_rcModes.isModeActive(MSP_Box::BOX_GPS_RESCUE)) {
        _flightModeFlags.set(GPS_RESCUE_MODE);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
    }
    if (_rcModes.isModeActive(MSP_Box::BOX_POSITION_HOLD) || _rcModes.isModeActive(MSP_Box::BOX_GPS_RESCUE)) {
        const FlightController::controls_t flightControls = _autopilot.calculateFlightControls(controls, _flightModeFlags.to_ulong());
        _flightController.updateSetpoints(flightControls, FlightController::FAILSAFE_OFF);
        return;
    }

    const float throttleStick = _rcModes.isModeActive(MSP_Box::BOX_ALTITUDE_HOLD) ? _autopilot.calculateThrottleForAltitudeHold(controls) : mapThrottle(controls.throttleStick);

    // map the radio controls to FlightController units
    const FlightController::controls_t flightControls = {
        .tickCount = controls.tickCount,
        .throttleStick = throttleStick,
        .rollStickDPS = applyRates(rates_t::ROLL, controls.rollStick),
        .pitchStickDPS = applyRates(rates_t::PITCH, controls.pitchStick),
        .yawStickDPS = applyRates(rates_t::YAW, controls.yawStick),
        .rollStickDegrees = controls.rollStick * _flightController.getMaxRollAngleDegrees(),
        .pitchStickDegrees = controls.pitchStick * _flightController.getMaxPitchAngleDegrees(),
        .controlMode = controlMode
    };

    _flightController.updateSetpoints(flightControls, FlightController::FAILSAFE_OFF);
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
            _flightController.updateSetpoints(flightControls, FlightController::FAILSAFE_ON);
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

void Cockpit::setRX_Config(const RX::config_t& rxConfig)
{
    _rxConfig = rxConfig;
}

void Cockpit::setRX_FailsafeChannelConfigs(const RX::failsafe_channel_configs_t& rxFailsafeChannelConfigs)
{
    _rxFailsafeChannelConfigs = rxFailsafeChannelConfigs;
}

void Cockpit::setRates(const rates_t& rates)
{
    _rates = rates;
    const float maxAngleRateRollDPS = applyRates(rates_t::ROLL, 1.0F);
    const float maxAngleRatePitchDPS = applyRates(rates_t::PITCH, 1.0F);
    const float maxAngleRateYawDPS = applyRates(rates_t::YAW, 1.0F);

    _flightController.setMaxAngleRates(maxAngleRateRollDPS, maxAngleRatePitchDPS, maxAngleRateYawDPS);
}

/*!
return state of given boxId box, handling ARM and FLIGHT_MODE
*/
bool Cockpit::getBoxIdState(MSP_Box::id_e boxId) const
{
    // we assume that all boxId below BOXID_FLIGHTMODE_LAST except BOXARM are mapped to flightmode

    if (boxId == MSP_Box::BOX_ARM) {
        return isArmed();
    }
    if (boxId < MSP_Box::BOX_ID_FLIGHTMODE_COUNT) {
        return _flightModeFlags.test(BoxIdToFlightModeMap[boxId]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    return _rcModes.isModeActive(boxId);
}

/*!
Pack used flightModeFlags into supplied bitset.
returns number of bits used
*/
size_t Cockpit::packFlightModeFlags(MSP_Box::bitset_t& flightModeFlags) const
{
    // Serialize the flags in the order we delivered them, ignoring BOX NAMES and BOX INDEXES
    flightModeFlags.reset();
    // map box_id_e enabled bits to MSP status indexes
    // only active boxIds are sent in status over MSP, other bits are not counted
    size_t boxIndex = 0;    // index of active boxId (matches sent permanentId and boxNames)
    for (int boxId = 0; boxId < MSP_Box::BOX_COUNT; ++boxId) {
        if (_mspBox.getActiveBoxId(static_cast<MSP_Box::id_e>(boxId))) {
            if (getBoxIdState(static_cast<MSP_Box::id_e>(boxId))) {
                flightModeFlags.set(boxIndex); // box is enabled
            }
            ++boxIndex; // box is active, count it
        }
    }
    // return count of used bits
    return boxIndex;
}
