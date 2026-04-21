#include "autopilot.h"
#include "cockpit.h"
#include "flight_controller.h"
#include "rc_modes.h"

#include <blackbox.h>
#include <debug.h>
#include <motor_mixer_base.h>
#include <receiver_base.h>

#include <cmath>

Cockpit::Cockpit(Autopilot& autopilot, [[maybe_unused]] const RcAdjustments::adjustment_configs_t* defaultAdjustment_configs) :
    _autopilot(autopilot)
#if defined(USE_RC_ADJUSTMENTS)
    ,_rc_adjustments(defaultAdjustment_configs)
#endif
{
    //!!flight_controller.set_yaw_spin_threshold_dps(1.25F*apply_rates(rates_t::YAW, 1.0F));

    _msp_box.set_active_box_id(MspBox::BOX_ARM);
    _msp_box.set_active_box_id(MspBox::BOX_PREARM);
    _msp_box.set_active_box_id(MspBox::BOX_AIRMODE);

    _msp_box.set_active_box_id(MspBox::BOX_ANTIGRAVITY);

    _msp_box.set_active_box_id(MspBox::BOX_ANGLE);
    _msp_box.set_active_box_id(MspBox::BOX_HORIZON);
    _msp_box.set_active_box_id(MspBox::BOX_ALTITUDE_HOLD);
    _msp_box.set_active_box_id(MspBox::BOX_HEADFREE);
    _msp_box.set_active_box_id(MspBox::BOX_HEADADJ);
    _msp_box.set_active_box_id(MspBox::BOX_FPV_ANGLE_MIX);
    if (feature_is_enabled(Features::FEATURE_INFLIGHT_ACC_CALIBRATE)) {
        _msp_box.set_active_box_id(MspBox::BOX_CALIBRATE);
    }
    _msp_box.set_active_box_id(MspBox::BOX_ACRO_TRAINER);

    _msp_box.set_active_box_id(MspBox::BOX_FAILSAFE);

    _msp_box.set_active_box_id(MspBox::BOX_BEEPER_ON);
    _msp_box.set_active_box_id(MspBox::BOX_BEEPER_MUTE);

    _msp_box.set_active_box_id(MspBox::BOX_PARALYZE);
    _msp_box.set_active_box_id(MspBox::BOX_MSP_OVERRIDE);

    _msp_box.set_active_box_id(MspBox::BOX_STICK_COMMAND_DISABLE);
    _msp_box.set_active_box_id(MspBox::BOX_READY);

    static_assert(_box_idToFlightModeMap[MspBox::BOX_ARM]           == 0); // not used
    static_assert(_box_idToFlightModeMap[MspBox::BOX_ANGLE]         == LOG2_ANGLE_MODE);
    static_assert(_box_idToFlightModeMap[MspBox::BOX_HORIZON]       == LOG2_HORIZON_MODE);
    static_assert(_box_idToFlightModeMap[MspBox::BOX_MAG]           == LOG2_MAG_MODE);
    static_assert(_box_idToFlightModeMap[MspBox::BOX_ALTITUDE_HOLD] == LOG2_ALTITUDE_HOLD_MODE);
    static_assert(_box_idToFlightModeMap[MspBox::BOX_POSITION_HOLD] == LOG2_POSITION_HOLD_MODE);
    static_assert(_box_idToFlightModeMap[MspBox::BOX_HEADFREE]      == LOG2_HEADFREE_MODE);
    static_assert(_box_idToFlightModeMap[MspBox::BOX_CHIRP]         == LOG2_CHIRP_MODE);
    static_assert(_box_idToFlightModeMap[MspBox::BOX_PASSTHRU]      == LOG2_PASSTHRU_MODE);
    static_assert(_box_idToFlightModeMap[MspBox::BOX_FAILSAFE]      == LOG2_FAILSAFE_MODE);
    static_assert(_box_idToFlightModeMap[MspBox::BOX_GPS_RESCUE]    == LOG2_GPS_RESCUE_MODE);
}

bool Cockpit::is_armed() const
{
    return _arming_flags & ARMED;
}

bool Cockpit::was_ever_armed() const
{
    return _arming_flags & WAS_EVER_ARMED;
}

void Cockpit::set_armed(FlightController& flight_controller, MotorMixerBase& motor_mixer)
{
    _arming_flags |= (ARMED | WAS_EVER_ARMED);
    flight_controller.motors_switch_on(motor_mixer);
}

void Cockpit::set_disarmed(FlightController& flight_controller, MotorMixerBase& motor_mixer)
{
    flight_controller.motors_switch_off(motor_mixer);
    _arming_flags &= ~ARMED;
    _failsafe.phase = FAILSAFE_DISARMED;
}

void Cockpit::set_arming_disabled_flag(uint32_t flag)
{
    _arming_disabled_flags |= flag;
}

void Cockpit::clear_arming_disabled_flag(uint32_t flag)
{
    _arming_disabled_flags &= ~flag;
}

uint32_t Cockpit::get_flight_mode_flags() const
{
    return _flight_mode_flags.to_ulong();
}

void Cockpit::set_rates_to_pass_through()
{
    _rates.rc_rates = { 100, 100, 100 }; // center sensitivity
    _rates.rc_expos = { 0, 0, 0}; // movement sensitivity, nonlinear
    _rates.rates   = { 0, 0, 0 }; // movement sensitivity, linear
    //_rates.ratesType = rates_t::RATES_TYPE_ACTUAL;
}

inline float clampToLimit(float value, uint16_t limit)
{
    const auto limitF = static_cast<float>(limit);
    return value < -limitF ? -limitF : value > limitF ? limitF : value;
}

float Cockpit::apply_rates(size_t axis, float rcCommand) const
{
    const float rcCommand2 = rcCommand * rcCommand;
    const float rcCommandAbs = std::fabs(rcCommand);

    float expo = _rates.rc_expos[axis] / 100.0F;
    expo = rcCommandAbs*rcCommand*(expo*(rcCommand2*rcCommand2 - 1.0F) + 1.0F);

    const float centerSensitivity = _rates.rc_rates[axis];
    const float stickMovement = std::fmaxf(0, _rates.rates[axis] - centerSensitivity);
    const float angleRate = 10.0F * (rcCommand*centerSensitivity + expo*stickMovement);
    //const float angleRate = 0.01F * (rcCommand*centerSensitivity + expo*stickMovement);

    return clampToLimit(angleRate, _rates.rate_limits[axis]);
}

/*!
Map throttle in range [-1.0F, 1.0F] to a parabolic curve
in the range [-_rates.throttle_limit_percent) / 100.0F, _rates.throttle_limit_percent) / 100.0F]
*/
float Cockpit::map_throttle(float throttle) const
{
    // alpha=0 gives a linear response, alpha=1 gives a parabolic (x^2) curve
    const float alpha = static_cast<float>(_rates.throttle_expo) / 255.0F;
    throttle *= 1.0F - alpha*(1.0F - (throttle < 0.0F ? -throttle : throttle));
    return throttle * static_cast<float>(_rates.throttle_limit_percent) / 100.0F;
}

void Cockpit::start_blackbox_recording(Blackbox* blackbox, FlightController& flight_controller, const MotorMixerBase& motor_mixer, const Debug& debug)
{
    if (blackbox) {
        blackbox->start(blackbox_start_t{
            .debug_mode = debug.get_mode(),
            .motor_count = static_cast<uint8_t>(motor_mixer.get_motor_count()),
            .servo_count = static_cast<uint8_t>(motor_mixer.get_servo_count())
        });
        flight_controller.set_blackbox_active(true);
    }
}

void Cockpit::stop_blackbox_recording(Blackbox* blackbox, FlightController& flight_controller)
{
    if (blackbox && flight_controller.is_blackbox_active()) {
        blackbox->finish();
        flight_controller.set_blackbox_active(false);
    }
}

void Cockpit::handle_arming_switch(FlightController& flight_controller, MotorMixerBase& motor_mixer, Blackbox* blackbox, const ReceiverBase& receiver, const RcModes& rc_modes, const Debug& debug)
{
#if defined(LIBRARY_RECEIVER_USE_ESPNOW) && !defined(TEST_FRAMEWORK)
    (void)rc_modes;
    if (receiver.get_switch(ReceiverBase::MOTOR_ON_OFF_SWITCH)) {
        _on_off_switch_pressed = true;
    } else {
        if (_on_off_switch_pressed) {
            // MOTOR_ON_OFF_SWITCH is false and _on_off_switch_pressed true means that the switch was previously pressed and is now being released
            _on_off_switch_pressed = false;
            // toggle arming when the onOff switch is released
            if (is_armed()) {
                set_disarmed(flight_controller, motor_mixer);
                stop_blackbox_recording(blackbox, flight_controller);
            } else {
                if (_record_to_blackbox_when_armed) {
                    start_blackbox_recording(blackbox, flight_controller, motor_mixer, debug);
                }
                set_armed(flight_controller, motor_mixer);
            }
        }
    }
#else
    (void)receiver;
    if (rc_modes.is_mode_active(MspBox::BOX_ARM)) {
        if (!is_armed()) {
            if (_record_to_blackbox_when_armed) {
                start_blackbox_recording(blackbox, flight_controller, motor_mixer, debug);
            }
            set_armed(flight_controller, motor_mixer);
        }
    } else {
        if (is_armed()) {
            set_disarmed(flight_controller, motor_mixer);
            stop_blackbox_recording(blackbox, flight_controller);
        }
    }
#endif
}

/*!
Called from Receiver Task.
*/
void Cockpit::update_controls(uint32_t tick_count, const ReceiverBase& receiver, receiver_context_t& ctx)
{
    const receiver_controls_t controls = receiver.get_controls();
    // failsafe handling
    _failsafe.phase = FAILSAFE_IDLE; // we've received a packet, so exit failsafe if we were in it
    _failsafe.tick_count = tick_count;

    // set the RC modes according to the receiver channel values
    ctx.rc_modes.update_activated_modes(receiver);

    handle_arming_switch(ctx.flight_controller, ctx.motor_mixer, ctx.blackbox, receiver, ctx.rc_modes, ctx.debug);

#if defined(USE_RC_ADJUSTMENTS)
    // process any in-flight adjustments
    if (!_cli_mode && !(ctx.rc_modes.is_mode_active(MspBox::BOX_PARALYZE) && !is_armed())) {
        _rc_adjustments.process_adjustments(receiver, ctx.flight_controller, ctx.blackbox, *this, ctx.osd, true); //!!TODO: check true parameter
    }
#endif

    _flight_mode_flags.reset();
    fc_control_mode_e control_mode = FC_CONTROL_MODE_RATE;
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_ANGLE)) {
        _flight_mode_flags.set(ANGLE_MODE);
        control_mode = FC_CONTROL_MODE_ANGLE;
    }
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_HORIZON)) {
        _flight_mode_flags.set(HORIZON_MODE);
        // we don't support horizon mode, instead we use the horizon mode setting to invoke level race mode
        control_mode = FC_CONTROL_MODE_LEVEL_RACE;
    }
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_ALTITUDE_HOLD)) {
        _flight_mode_flags.set(ALTITUDE_HOLD_MODE);
        control_mode = FC_CONTROL_MODE_ANGLE;
        // not currently in altitude hold mode, so set the altitude hold setpoint
        if (!_autopilot.isAltitudeHoldSetpointSet()) {
            _autopilot.setAltitudeHoldSetpoint();
        }
    }
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_POSITION_HOLD)) {
        _flight_mode_flags.set(POSITION_HOLD_MODE);
        control_mode = FC_CONTROL_MODE_ANGLE;
    }
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_MAG)) {
        _flight_mode_flags.set(MAG_MODE);
    }
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_HEADFREE)) {
        _flight_mode_flags.set(HEADFREE_MODE);
    }
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_CHIRP)) {
        _flight_mode_flags.set(CHIRP_MODE);
    }
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_PASSTHRU)) {
        _flight_mode_flags.set(PASSTHRU_MODE);
    }
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_FAILSAFE)) {
        _flight_mode_flags.set(FAILSAFE_MODE);
        control_mode = FC_CONTROL_MODE_ANGLE;
    }
    if (ctx.rc_modes.is_mode_active(MspBox::BOX_GPS_RESCUE)) {
        _flight_mode_flags.set(GPS_RESCUE_MODE);
        control_mode = FC_CONTROL_MODE_ANGLE;
    }
    _gps_rescue_configured =  _failsafe_config.procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || ctx.rc_modes.is_mode_activation_condition_present(MspBox::BOX_GPS_RESCUE);

    if (ctx.rc_modes.is_mode_active(MspBox::BOX_POSITION_HOLD) || ctx.rc_modes.is_mode_active(MspBox::BOX_GPS_RESCUE)) {
        const fc_controls_t flightControls = _autopilot.calculateFlightControls(controls, _flight_mode_flags.to_ulong());
        ctx.flight_controller.update_setpoints(flightControls, FlightController::FAILSAFE_OFF, ctx.debug);
        return;
    }

    const float throttle_stick = ctx.rc_modes.is_mode_active(MspBox::BOX_ALTITUDE_HOLD) ? _autopilot.calculateThrottleForAltitudeHold(controls) : map_throttle(controls.throttle);

    // map the radio controls to FlightController units
    const fc_controls_t flightControls = {
        .tick_count = tick_count,
        .throttle_stick = throttle_stick,
        .roll_stick_dps = apply_rates(rates_t::ROLL, controls.roll),
        .pitch_stick_dps = apply_rates(rates_t::PITCH, controls.pitch),
        .yaw_stick_dps = apply_rates(rates_t::YAW, controls.yaw),
        .roll_stick_degrees = controls.roll * ctx.flight_controller.get_max_roll_angle_degrees(),
        .pitch_stick_degrees = controls.pitch * ctx.flight_controller.get_max_pitch_angle_degrees(),
        .control_mode = control_mode
    };

    ctx.flight_controller.update_setpoints(flightControls, FlightController::FAILSAFE_OFF, ctx.debug);
}

void Cockpit::check_failsafe(uint32_t tick_count, receiver_context_t& ctx)
{
    ctx.flight_controller.detect_crash_or_spin();

    if ((tick_count - _failsafe.tick_count > _failsafe.tick_count_threshold) && (_failsafe.phase != FAILSAFE_DISARMED)) {
        // We've had tick_count_threshold ticks without a packet, so we seem to have lost contact with the transmitter,
        if ((tick_count - _failsafe.tick_count < _failsafe.tick_count_switch_off_threshold)) {
            // failsafe detected, so zero all sticks, set throttle to its failsafe value, and switch to angle mode
            _failsafe.phase = FAILSAFE_RX_LOSS_DETECTED;
            const fc_controls_t flightControls = {
                .tick_count = tick_count,
                .throttle_stick = (static_cast<float>(_failsafe_config.throttle_pwm) - ReceiverBase::CHANNEL_LOW_F) / ReceiverBase::CHANNEL_RANGE_F,
                .roll_stick_dps = 0.0F,
                .pitch_stick_dps = 0.0F,
                .yaw_stick_dps = 0.0F,
                .roll_stick_degrees = 0.0F,
                .pitch_stick_degrees = 0.0F,
                .control_mode = FC_CONTROL_MODE_ANGLE
            };
            ctx.flight_controller.update_setpoints(flightControls, FlightController::FAILSAFE_ON, ctx.debug);
        } else {
            // we've lost contact for an extended period, so disarm.
            _failsafe.phase = FAILSAFE_DISARMED;
            set_disarmed(ctx.flight_controller, ctx.motor_mixer);
        }
    }
}

void Cockpit::set_failsafe_config(const failsafe_config_t& failsafe_config)
{
    _failsafe_config = failsafe_config;
}

bool Cockpit::gps_rescue_is_configured() const
{
    return _gps_rescue_configured;
}


void Cockpit::set_rx_config(const rx_config_t& rx_config)
{
    _rx_config = rx_config;
}

void Cockpit::set_rx_failsafe_channel_configs(const rx_failsafe_channel_configs_t& rx_failsafe_channel_configs)
{
    _rx_failsafe_channel_configs = rx_failsafe_channel_configs;
}

void Cockpit::set_rates(const rates_t& rates) // NOLINT(readability-make-member-function-const)
{
    _rates = rates;
    //const float max_angleRateRollDPS = apply_rates(rates_t::ROLL, 1.0F);
    //const float max_angleRatePitchDPS = apply_rates(rates_t::PITCH, 1.0F);
    //const float max_angleRateYawDPS = apply_rates(rates_t::YAW, 1.0F);

    //!!_flight_controller.set_max_angle_rates(max_angleRateRollDPS, max_angleRatePitchDPS, max_angleRateYawDPS);
}

/*!
return state of given box_id box, handling ARM and FLIGHT_MODE
*/
bool Cockpit::get_box_id_state(uint8_t box_id, const RcModes& rc_modes) const
{
    // we assume that all box_id below BOXID_FLIGHTMODE_LAST except BOXARM are mapped to flightmode

    if (box_id == MspBox::BOX_ARM) {
        return is_armed();
    }
    if (box_id < MspBox::BOX_ID_FLIGHTMODE_COUNT) {
        return _flight_mode_flags.test(_box_idToFlightModeMap[box_id]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    return rc_modes.is_mode_active(box_id);
}

/*!
Pack used flight_mode_flags into supplied bitset.
returns number of bits used
*/
size_t Cockpit::pack_flight_mode_flags(std::bitset<MSP_BOX_COUNT>& flight_mode_flags, const RcModes& rc_modes) const
{
    // Serialize the flags in the order we delivered them, ignoring BOX NAMES and BOX INDEXES
    flight_mode_flags.reset();
    // map box_id_e enabled bits to MSP status indexes
    // only active box_ids are sent in status over MSP, other bits are not counted
    size_t box_index = 0;    // index of active box_id (matches sent permanentId and boxNames)
    for (uint8_t box_id = 0; box_id < MSP_BOX_COUNT; ++box_id) {
        if (_msp_box.get_active_box_id(box_id)) {
            if (get_box_id_state(box_id, rc_modes)) {
                flight_mode_flags.set(box_index); // box is enabled
            }
            ++box_index; // box is active, count it
        }
    }
    // return count of used bits
    return box_index;
}
