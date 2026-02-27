#include "Autopilot.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "RC_Modes.h"

#include <blackbox.h>
#include <debug.h>
#include <motor_mixer_base.h>
#include <receiver_base.h>

#include <cmath>

Cockpit::Cockpit(Autopilot& autopilot, [[maybe_unused]] const RC_Adjustments::adjustment_configs_t* defaultAdjustmentConfigs) :
    _autopilot(autopilot)
#if defined(USE_RC_ADJUSTMENTS)
    ,_rcAdjustments(defaultAdjustmentConfigs)
#endif
{
    //!!flightController.setYawSpinThresholdDPS(1.25F*applyRates(rates_t::YAW, 1.0F));

    _mspBox.set_active_box_id(MspBox::BOX_ARM);
    _mspBox.set_active_box_id(MspBox::BOX_PREARM);
    _mspBox.set_active_box_id(MspBox::BOX_AIRMODE);

    _mspBox.set_active_box_id(MspBox::BOX_ANTIGRAVITY);

    _mspBox.set_active_box_id(MspBox::BOX_ANGLE);
    _mspBox.set_active_box_id(MspBox::BOX_HORIZON);
    _mspBox.set_active_box_id(MspBox::BOX_ALTITUDE_HOLD);
    _mspBox.set_active_box_id(MspBox::BOX_HEADFREE);
    _mspBox.set_active_box_id(MspBox::BOX_HEADADJ);
    _mspBox.set_active_box_id(MspBox::BOX_FPV_ANGLE_MIX);
    if (featureIsEnabled(Features::FEATURE_INFLIGHT_ACC_CALIBRATE)) {
        _mspBox.set_active_box_id(MspBox::BOX_CALIBRATE);
    }
    _mspBox.set_active_box_id(MspBox::BOX_ACRO_TRAINER);

    _mspBox.set_active_box_id(MspBox::BOX_FAILSAFE);

    _mspBox.set_active_box_id(MspBox::BOX_BEEPER_ON);
    _mspBox.set_active_box_id(MspBox::BOX_BEEPER_MUTE);

    _mspBox.set_active_box_id(MspBox::BOX_PARALYZE);
    _mspBox.set_active_box_id(MspBox::BOX_MSP_OVERRIDE);

    _mspBox.set_active_box_id(MspBox::BOX_STICK_COMMAND_DISABLE);
    _mspBox.set_active_box_id(MspBox::BOX_READY);

    static_assert(BoxIdToFlightModeMap[MspBox::BOX_ARM]           == 0); // not used
    static_assert(BoxIdToFlightModeMap[MspBox::BOX_ANGLE]         == LOG2_ANGLE_MODE);
    static_assert(BoxIdToFlightModeMap[MspBox::BOX_HORIZON]       == LOG2_HORIZON_MODE);
    static_assert(BoxIdToFlightModeMap[MspBox::BOX_MAG]           == LOG2_MAG_MODE);
    static_assert(BoxIdToFlightModeMap[MspBox::BOX_ALTITUDE_HOLD] == LOG2_ALTITUDE_HOLD_MODE);
    static_assert(BoxIdToFlightModeMap[MspBox::BOX_POSITION_HOLD] == LOG2_POSITION_HOLD_MODE);
    static_assert(BoxIdToFlightModeMap[MspBox::BOX_HEADFREE]      == LOG2_HEADFREE_MODE);
    static_assert(BoxIdToFlightModeMap[MspBox::BOX_CHIRP]         == LOG2_CHIRP_MODE);
    static_assert(BoxIdToFlightModeMap[MspBox::BOX_PASSTHRU]      == LOG2_PASSTHRU_MODE);
    static_assert(BoxIdToFlightModeMap[MspBox::BOX_FAILSAFE]      == LOG2_FAILSAFE_MODE);
    static_assert(BoxIdToFlightModeMap[MspBox::BOX_GPS_RESCUE]    == LOG2_GPS_RESCUE_MODE);
}

bool Cockpit::isArmed() const
{
    return _armingFlags & ARMED;
}

bool Cockpit::wasEverArmed() const
{
    return _armingFlags & WAS_EVER_ARMED;
}

void Cockpit::setArmed(FlightController& flightController, MotorMixerBase& motorMixer)
{
    _armingFlags |= (ARMED | WAS_EVER_ARMED);
    flightController.motorsSwitchOn(motorMixer);
}

void Cockpit::setDisarmed(FlightController& flightController, MotorMixerBase& motorMixer)
{
    flightController.motorsSwitchOff(motorMixer);
    _armingFlags &= ~ARMED;
    _failsafe.phase = FAILSAFE_DISARMED;
}

void Cockpit::setArmingDisabledFlag(uint32_t flag)
{
    _armingDisabledFlags |= flag;
}

void Cockpit::clearArmingDisabledFlag(uint32_t flag)
{
    _armingDisabledFlags &= ~flag;
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

void Cockpit::startBlackboxRecording(Blackbox* blackbox, FlightController& flightController, const MotorMixerBase& motorMixer, const Debug& debug)
{
    if (blackbox) {
        blackbox->start(Blackbox::start_t{
            .debugMode = static_cast<uint16_t>(debug.getMode()),
            .motorCount = static_cast<uint8_t>(motorMixer.get_motor_count()),
            .servoCount = static_cast<uint8_t>(motorMixer.get_servo_count())
        });
        flightController.setBlackboxActive(true);
    }
}

void Cockpit::stopBlackboxRecording(Blackbox* blackbox, FlightController& flightController)
{
    if (blackbox && flightController.isBlackboxActive()) {
        blackbox->finish();
        flightController.setBlackboxActive(false);
    }
}

void Cockpit::handleArmingSwitch(FlightController& flightController, MotorMixerBase& motorMixer, Blackbox* blackbox, const ReceiverBase& receiver, const RcModes& rc_modes, const Debug& debug)
{
#if defined(LIBRARY_RECEIVER_USE_ESPNOW) && !defined(TEST_FRAMEWORK)
    (void)rc_modes;
    if (receiver.get_switch(ReceiverBase::MOTOR_ON_OFF_SWITCH)) {
        _onOffSwitchPressed = true;
    } else {
        if (_onOffSwitchPressed) {
            // MOTOR_ON_OFF_SWITCH is false and _onOffSwitchPressed true means that the switch was previously pressed and is now being released
            _onOffSwitchPressed = false;
            // toggle arming when the onOff switch is released
            if (isArmed()) {
                setDisarmed(flightController, motorMixer);
                stopBlackboxRecording(blackbox, flightController);
            } else {
                if (_recordToBlackboxWhenArmed) {
                    startBlackboxRecording(blackbox, flightController, motorMixer, debug);
                }
                setArmed(flightController, motorMixer);
            }
        }
    }
#else
    (void)receiver;
    if (rc_modes.is_mode_active(MspBox::BOX_ARM)) {
        if (!isArmed()) {
            if (_recordToBlackboxWhenArmed) {
                startBlackboxRecording(blackbox, flightController, motorMixer, debug);
            }
            setArmed(flightController, motorMixer);
        }
    } else {
        if (isArmed()) {
            setDisarmed(flightController, motorMixer);
            stopBlackboxRecording(blackbox, flightController);
        }
    }
#endif
}

/*!
Called from Receiver Task.
*/
void Cockpit::update_controls(uint32_t tick_count, const ReceiverBase& receiver, receiver_parameter_group_t& pg)
{
    const receiver_controls_t controls = receiver.get_controls();
    // failsafe handling
    _failsafe.phase = FAILSAFE_IDLE; // we've received a packet, so exit failsafe if we were in it
    _failsafe.tick_count = tick_count;

    // set the RC modes according to the receiver channel values
    pg.rc_modes.update_activated_modes(receiver);

    handleArmingSwitch(pg.flight_controller, pg.motor_mixer, pg.blackbox, receiver, pg.rc_modes, pg.debug);

#if defined(USE_RC_ADJUSTMENTS)
    // process any in-flight adjustments
    if (!_cliMode && !(pg.rc_modes.is_mode_active(MspBox::BOX_PARALYZE) && !isArmed())) {
        _rcAdjustments.processAdjustments(receiver, pg.flight_controller, pg.blackbox, *this, pg.osd, true); //!!TODO: check true parameter
    }
#endif

    _flightModeFlags.reset();
    FlightController::control_mode_e controlMode = FlightController::CONTROL_MODE_RATE;
    if (pg.rc_modes.is_mode_active(MspBox::BOX_ANGLE)) {
        _flightModeFlags.set(ANGLE_MODE);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
    }
    if (pg.rc_modes.is_mode_active(MspBox::BOX_HORIZON)) {
        _flightModeFlags.set(HORIZON_MODE);
        // we don't support horizon mode, instead we use the horizon mode setting to invoke level race mode
        controlMode = FlightController::CONTROL_MODE_LEVEL_RACE;
    }
    if (pg.rc_modes.is_mode_active(MspBox::BOX_ALTITUDE_HOLD)) {
        _flightModeFlags.set(ALTITUDE_HOLD_MODE);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
        // not currently in altitude hold mode, so set the altitude hold setpoint
        if (!_autopilot.isAltitudeHoldSetpointSet()) {
            _autopilot.setAltitudeHoldSetpoint();
        }
    }
    if (pg.rc_modes.is_mode_active(MspBox::BOX_POSITION_HOLD)) {
        _flightModeFlags.set(POSITION_HOLD_MODE);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
    }
    if (pg.rc_modes.is_mode_active(MspBox::BOX_MAG)) {
        _flightModeFlags.set(MAG_MODE);
    }
    if (pg.rc_modes.is_mode_active(MspBox::BOX_HEADFREE)) {
        _flightModeFlags.set(HEADFREE_MODE);
    }
    if (pg.rc_modes.is_mode_active(MspBox::BOX_CHIRP)) {
        _flightModeFlags.set(CHIRP_MODE);
    }
    if (pg.rc_modes.is_mode_active(MspBox::BOX_PASSTHRU)) {
        _flightModeFlags.set(PASSTHRU_MODE);
    }
    if (pg.rc_modes.is_mode_active(MspBox::BOX_FAILSAFE)) {
        _flightModeFlags.set(FAILSAFE_MODE);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
    }
    if (pg.rc_modes.is_mode_active(MspBox::BOX_GPS_RESCUE)) {
        _flightModeFlags.set(GPS_RESCUE_MODE);
        controlMode = FlightController::CONTROL_MODE_ANGLE;
    }
    _gpsRescueConfigured =  _failsafeConfig.procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || pg.rc_modes.is_mode_activation_condition_present(MspBox::BOX_GPS_RESCUE);

    if (pg.rc_modes.is_mode_active(MspBox::BOX_POSITION_HOLD) || pg.rc_modes.is_mode_active(MspBox::BOX_GPS_RESCUE)) {
        const FlightController::controls_t flightControls = _autopilot.calculateFlightControls(controls, _flightModeFlags.to_ulong());
        pg.flight_controller.updateSetpoints(flightControls, FlightController::FAILSAFE_OFF, pg.debug);
        return;
    }

    const float throttleStick = pg.rc_modes.is_mode_active(MspBox::BOX_ALTITUDE_HOLD) ? _autopilot.calculateThrottleForAltitudeHold(controls) : mapThrottle(controls.throttle);

    // map the radio controls to FlightController units
    const FlightController::controls_t flightControls = {
        .tick_count = tick_count,
        .throttleStick = throttleStick,
        .rollStickDPS = applyRates(rates_t::ROLL, controls.roll),
        .pitchStickDPS = applyRates(rates_t::PITCH, controls.pitch),
        .yawStickDPS = applyRates(rates_t::YAW, controls.yaw),
        .rollStickDegrees = controls.roll * pg.flight_controller.getMaxRollAngleDegrees(),
        .pitchStickDegrees = controls.pitch * pg.flight_controller.getMaxPitchAngleDegrees(),
        .controlMode = controlMode
    };

    pg.flight_controller.updateSetpoints(flightControls, FlightController::FAILSAFE_OFF, pg.debug);
}

void Cockpit::check_failsafe(uint32_t tick_count, receiver_parameter_group_t& pg)
{
    pg.flight_controller.detectCrashOrSpin();

    if ((tick_count - _failsafe.tick_count > _failsafe.tick_countThreshold) && (_failsafe.phase != FAILSAFE_DISARMED)) {
        // We've had tick_countThreshold ticks without a packet, so we seem to have lost contact with the transmitter,
        if ((tick_count - _failsafe.tick_count < _failsafe.tick_countSwitchOffThreshold)) {
            // failsafe detected, so zero all sticks, set throttle to its failsafe value, and switch to angle mode
            _failsafe.phase = FAILSAFE_RX_LOSS_DETECTED;
            const FlightController::controls_t flightControls = {
                .tick_count = tick_count,
                .throttleStick = (static_cast<float>(_failsafeConfig.throttle_pwm) - ReceiverBase::CHANNEL_LOW_F) / ReceiverBase::CHANNEL_RANGE_F,
                .rollStickDPS = 0.0F,
                .pitchStickDPS = 0.0F,
                .yawStickDPS = 0.0F,
                .rollStickDegrees = 0.0F,
                .pitchStickDegrees = 0.0F,
                .controlMode = FlightController::CONTROL_MODE_ANGLE
            };
            pg.flight_controller.updateSetpoints(flightControls, FlightController::FAILSAFE_ON, pg.debug);
        } else {
            // we've lost contact for an extended period, so disarm.
            _failsafe.phase = FAILSAFE_DISARMED;
            setDisarmed(pg.flight_controller, pg.motor_mixer);
        }
    }
}

void Cockpit::setFailsafeConfig(const failsafe_config_t& failsafeConfig)
{
    _failsafeConfig = failsafeConfig;
}

bool Cockpit::gpsRescueIsConfigured() const
{
    return _gpsRescueConfigured;
}


void Cockpit::setRX_Config(const rx_config_t& rxConfig)
{
    _rxConfig = rxConfig;
}

void Cockpit::setRX_FailsafeChannelConfigs(const RX::failsafe_channel_configs_t& rxFailsafeChannelConfigs)
{
    _rxFailsafeChannelConfigs = rxFailsafeChannelConfigs;
}

void Cockpit::setRates(const rates_t& rates) // NOLINT(readability-make-member-function-const)
{
    _rates = rates;
    //const float maxAngleRateRollDPS = applyRates(rates_t::ROLL, 1.0F);
    //const float maxAngleRatePitchDPS = applyRates(rates_t::PITCH, 1.0F);
    //const float maxAngleRateYawDPS = applyRates(rates_t::YAW, 1.0F);

    //!!_flightController.setMaxAngleRates(maxAngleRateRollDPS, maxAngleRatePitchDPS, maxAngleRateYawDPS);
}

/*!
return state of given boxId box, handling ARM and FLIGHT_MODE
*/
bool Cockpit::getBoxIdState(uint8_t boxId, const RcModes& rc_modes) const
{
    // we assume that all boxId below BOXID_FLIGHTMODE_LAST except BOXARM are mapped to flightmode

    if (boxId == MspBox::BOX_ARM) {
        return isArmed();
    }
    if (boxId < MspBox::BOX_ID_FLIGHTMODE_COUNT) {
        return _flightModeFlags.test(BoxIdToFlightModeMap[boxId]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    return rc_modes.is_mode_active(boxId);
}

/*!
Pack used flightModeFlags into supplied bitset.
returns number of bits used
*/
size_t Cockpit::packFlightModeFlags(MspBox::bitset_t& flightModeFlags, const RcModes& rc_modes) const
{
    // Serialize the flags in the order we delivered them, ignoring BOX NAMES and BOX INDEXES
    flightModeFlags.reset();
    // map box_id_e enabled bits to MSP status indexes
    // only active boxIds are sent in status over MSP, other bits are not counted
    size_t boxIndex = 0;    // index of active boxId (matches sent permanentId and boxNames)
    for (uint8_t boxId = 0; boxId < MspBox::BOX_COUNT; ++boxId) {
        if (_mspBox.get_active_box_id(boxId)) {
            if (getBoxIdState(boxId, rc_modes)) {
                flightModeFlags.set(boxIndex); // box is enabled
            }
            ++boxIndex; // box is active, count it
        }
    }
    // return count of used bits
    return boxIndex;
}
