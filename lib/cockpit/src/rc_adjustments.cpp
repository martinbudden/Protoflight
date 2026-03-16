#include "cockpit.h"
#include "flight_controller.h"
#include "rc_adjustments.h"
#if defined (USE_OSD_PROFILES)
#include "osd.h"
#endif
#include "rates.h"

#include <blackbox.h>
#include <cassert>
#include <cstring>
#include <receiver_base.h>
#include <time_microseconds.h>


RcAdjustments::RcAdjustments(const adjustment_configs_t* defaultAdjustment_configs) :
    _default_adjustment_configs(defaultAdjustment_configs)
{
#if !defined(FRAMEWORK_TEST)
    assert(defaultAdjustment_configs != nullptr);
#endif
}

void RcAdjustments::beeper_confirmation_beeps(uint8_t beepCount)
{
    (void)beepCount;
}

void RcAdjustments::set_adjustment_ranges(const rc_adjustment_ranges_t& adjustment_ranges)
{
    _adjustment_ranges = adjustment_ranges;
}

const rc_adjustment_ranges_t& RcAdjustments::get_adjustment_ranges() const
{
    return _adjustment_ranges;
}

void RcAdjustments::set_adjustment_configs(const adjustment_configs_t& adjustment_configs)
{
    _adjustment_configs = adjustment_configs;
}

const RcAdjustments::adjustment_configs_t& RcAdjustments::get_adjustment_configs() const
{
    return _adjustment_configs;
}

const rc_adjustment_range_t& RcAdjustments::get_adjustment_range(size_t index) const
{
    assert(index < RC_MAX_ADJUSTMENT_RANGE_COUNT && "Adjustment Range Index too large");
    return _adjustment_ranges[index];
}

void RcAdjustments::set_adjustment_range(size_t index, const rc_adjustment_range_t& adjustment_range)
{
    if (index < RC_MAX_ADJUSTMENT_RANGE_COUNT) {
        _adjustment_ranges[index] = adjustment_range;
    }
}

void RcAdjustments::active_adjustment_range_reset()
{
    _stepwise_adjustment_count = ADJUSTMENT_RANGE_COUNT_INVALID;
}

void RcAdjustments::blackbox_log_inflight_adjustment_event(Blackbox* blackbox, adjustment_e adjustment, int32_t newValue)
{
#if defined(USE_BLACKBOX)
    if (blackbox) {
        Blackbox::log_event_data_u logEvent {
            .inflight_adjustment = {
                .new_value = newValue,
                .new_float_value =  0.0F,
                .adjustment = adjustment,
                .float_flag = false
            }
        };
        blackbox->log_event(Blackbox::LOG_EVENT_INFLIGHT_ADJUSTMENT, &logEvent);
    }
#else
    (void)blackbox;
    (void)adjustment;
    (void)newValue;
#endif
}

int32_t RcAdjustments::apply_step_adjustment(FlightController& flight_controller, Blackbox* blackbox, rates_t& rates, adjustment_e adjustment, int32_t delta)
{
    beeper_confirmation_beeps(delta > 0 ? 2 : 1);

    switch (adjustment) {
    case ADJUSTMENT_RC_RATE:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_ROLL_RC_RATE: {
        //newValue = constrain((int)controlRateConfig->rc_rates[FD_ROLL] + delta, 1, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rc_rates[FlightController::FD_ROLL] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rc_rates[FlightController::FD_ROLL] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_RC_RATE, newValue);
        if (adjustment == ADJUSTMENT_ROLL_RC_RATE) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_PITCH_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rc_rates[FlightController::FD_PITCH] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rc_rates[FlightController::FD_PITCH] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_RC_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_RC_EXPO:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_ROLL_RC_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rc_expos[FlightController::FD_ROLL] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rc_expos[FlightController::FD_ROLL] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_RC_EXPO, newValue);
        if (adjustment == ADJUSTMENT_ROLL_RC_EXPO) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_PITCH_RC_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rc_expos[FlightController::FD_PITCH] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rc_expos[FlightController::FD_PITCH] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_RC_EXPO, newValue);
        return newValue;
    }
    case ADJUSTMENT_THROTTLE_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.throttle_expo + delta), uint8_t{0}, uint8_t{rates_t::THROTTLE_MAX});
        rates.throttle_expo = newValue;
        //!!initRcProcessing();
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_THROTTLE_EXPO, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_RATE:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rates[FlightController::FD_PITCH] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[FlightController::FD_PITCH] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_RATE, newValue);
        if (adjustment == ADJUSTMENT_PITCH_RATE) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rates[FlightController::FD_ROLL] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[FlightController::FD_ROLL] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rates[FlightController::FD_YAW] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[FlightController::FD_YAW] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_YAW_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_P:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_P: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::PITCH_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kp + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_p_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_P, newValue);
        if (adjustment == ADJUSTMENT_PITCH_P) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_P: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::ROLL_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kp + delta), uint16_t{0}, FlightController::PID_GAIN_MAX); // FIXME magic numbers repeated in cli.c
        flight_controller.set_pid_p_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_P, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_I:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_I: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::PITCH_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.ki + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_i_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_I, newValue);
        if (adjustment == ADJUSTMENT_PITCH_I) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_I: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::ROLL_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.ki + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_i_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_I, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_D:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_D: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::PITCH_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kd + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_d_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_D, newValue);
        if (adjustment == ADJUSTMENT_PITCH_D) {
            return newValue;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_D
        [[fallthrough]];
    }
    case ADJUSTMENT_ROLL_D: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::ROLL_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kd + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_d_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_D, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_P: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::YAW_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kp + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_p_msp(FlightController::YAW_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_YAW_P, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_I: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::YAW_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.ki + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_i_msp(FlightController::YAW_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_YAW_I, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_D:
        return -1;
    case ADJUSTMENT_PITCH_ROLL_K:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_K: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::PITCH_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kk + delta), uint16_t{0}, FlightController::K_GAIN_MAX);
        flight_controller.set_pid_k_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_K, newValue);
        if (adjustment == ADJUSTMENT_PITCH_K) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_K: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::ROLL_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kk + delta), uint16_t{0}, FlightController::K_GAIN_MAX);
        flight_controller.set_pid_k_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_K, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_K: {
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::YAW_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kk + delta), uint16_t{0}, FlightController::K_GAIN_MAX);
        flight_controller.set_pid_k_msp(FlightController::YAW_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_YAW_K, newValue);
        return newValue;
    }
#if defined(USE_FEEDFORWARD)
    case ADJUSTMENT_FEEDFORWARD_TRANSITION: {
        newValue = constrain(current_pid_profile->feedforward_transition + delta, 1, 100); // FIXME magic numbers repeated in cli.c
        current_pid_profile->feedforward_transition = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_FEEDFORWARD_TRANSITION, newValue);
        return newValue;
    }
#endif
    default:
        return -1;
    };
}

int32_t RcAdjustments::apply_absolute_adjustment(FlightController& flight_controller, Blackbox* blackbox, rates_t& rates, adjustment_e adjustment, int32_t value)
{
    switch (adjustment) {
    case ADJUSTMENT_RC_RATE:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_ROLL_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rc_rates[FlightController::FD_ROLL] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_RC_RATE, newValue);
        if (adjustment == ADJUSTMENT_ROLL_RC_RATE) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_PITCH_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rc_rates[FlightController::FD_PITCH] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_RC_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rc_rates[FlightController::FD_YAW] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_YAW_RC_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_RC_EXPO:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_ROLL_RC_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_EXPOS_MAX});
        rates.rc_expos[rates_t::ROLL] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_RC_EXPO, newValue);
        if (adjustment == ADJUSTMENT_ROLL_RC_EXPO) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_PITCH_RC_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_EXPOS_MAX});
        rates.rc_expos[rates_t::PITCH] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_RC_EXPO, newValue);
        return newValue;
    }
    case ADJUSTMENT_THROTTLE_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{0}, uint8_t{rates_t::RC_EXPOS_MAX});
        rates.throttle_expo = newValue;
        //initRcProcessing();
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_THROTTLE_EXPO, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_RATE:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{0}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[rates_t::PITCH] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_RATE, newValue);
        if (adjustment == ADJUSTMENT_PITCH_RATE) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{0}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[rates_t::ROLL] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{0}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[rates_t::YAW] = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_YAW_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_P:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_P: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_p_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_P, newValue);
        if (adjustment == ADJUSTMENT_PITCH_P) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_P: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_p_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_P, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_I:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_I: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_i_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_I, newValue);
        if (adjustment == ADJUSTMENT_PITCH_I) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_I: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_i_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_I, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_D:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_D: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_d_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_D, newValue);
        if (adjustment == ADJUSTMENT_PITCH_D) {
            return newValue;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_D
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_D: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_d_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_D, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_P: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_p_msp(FlightController::YAW_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_YAW_P, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_I: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flight_controller.set_pid_i_msp(FlightController::YAW_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_YAW_I, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_D:
        return -1;
    case ADJUSTMENT_PITCH_ROLL_K:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_K: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::K_GAIN_MAX);
        flight_controller.set_pid_k_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_PITCH_K, newValue);
        if (adjustment == ADJUSTMENT_PITCH_K) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_K: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::K_GAIN_MAX);
        flight_controller.set_pid_k_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_ROLL_K, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_K: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::K_GAIN_MAX);
        flight_controller.set_pid_k_msp(FlightController::YAW_RATE_DPS, newValue);
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_YAW_K, newValue);
        return newValue;
    }
#if defined(USE_FEEDFORWARD)
    case ADJUSTMENT_FEEDFORWARD_TRANSITION: {
        newValue = constrain(value, 1, 100); // FIXME magic numbers repeated in cli.c
        current_pid_profile->feedforward_transition = newValue;
        blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_FEEDFORWARD_TRANSITION, newValue);
        return newValue;
    }
#endif
    default:
        return -1;
    };
}

uint8_t RcAdjustments::apply_select_adjustment(FlightController& flight_controller, Cockpit& cockpit, Blackbox* blackbox, [[maybe_unused]] OSD* osd, adjustment_e adjustment, uint8_t position)
{
    (void)cockpit;
    uint8_t beeps = 0;

    switch (adjustment) {
    /*!!case ADJUSTMENT_RATE_PROFILE:
        if (cockpit.get_current_rate_profile_index() != position) {
            cockpit.set_current_rate_profile_index(position);
            blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_RATE_PROFILE, position);
            beeps = position + 1;
        }
        break;*/
    case ADJUSTMENT_HORIZON_STRENGTH: {
        const uint8_t newValue = std::clamp(position, uint8_t{0}, uint8_t{FlightController::PID_GAIN_MAX}); // FIXME magic numbers repeated in serial_cli.c
        const pid_constants_uint16_t pid = flight_controller.get_pid_msp(FlightController::ROLL_ANGLE_DEGREES);
        if (pid.kd != newValue) {
            beeps = static_cast<uint8_t>((newValue - pid.kd) / 8) + 1;
            flight_controller.set_pid_p_msp(FlightController::ROLL_ANGLE_DEGREES, newValue);
            blackbox_log_inflight_adjustment_event(blackbox, ADJUSTMENT_HORIZON_STRENGTH, position);
        }
        break;
    }
#if defined(USE_PID_AUDIO)
    case ADJUSTMENT_PID_AUDIO:
        {
            pidAudioModes_e newMode = pidAudioPositionToModeMap[position];
            if (newMode != pidAudioGetMode()) {
                pidAudioSetMode(newMode);
            }
        }
        break;
#endif
#if defined (USE_OSD_PROFILES)
    case ADJUSTMENT_OSD_PROFILE:
        if (osd != nullptr && osd->get_profile() != (position + 1)) {
            osd->set_profile(position + 1);
        }
        break;
#endif
#if defined(USE_LED_STRIP)
    case ADJUSTMENT_LED_PROFILE:
        if (getLedProfile() != position) {
            setLedProfile(position);
        }
        break;
    case ADJUSTMENT_LED_DIMMER:
        if (getLedBrightness() != position) {
            setLedBrightness(position);
        }
        break;
#endif
    default:
        break;
    }

    if (beeps) {
        beeper_confirmation_beeps(beeps);
    }
    return position;
}

void RcAdjustments::calculate_active_adjustment_ranges()
{
    rc_adjustment_range_t defaultAdjustmentRange {};

    _stepwise_adjustment_count = 0;
    _continuos_adjustment_count = 0;
    uint8_t ii = 0;
    for (rc_adjustment_range_t& adjustment_range: _adjustment_ranges) {
        if (memcmp(&adjustment_range, &defaultAdjustmentRange, sizeof(defaultAdjustmentRange)) != 0) {
            const adjustment_config_t& adjustment_config = (*_default_adjustment_configs)[adjustment_range.adjustment_config - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
            if (adjustment_range.adjustmentCenter == 0 && adjustment_config.mode != ADJUSTMENT_MODE_SELECT) {
                timed_adjustment_state_t& adjustmentState = _stepwise_adjustments[static_cast<size_t>(_stepwise_adjustment_count++)];
                adjustmentState.adjustment_range_index = ii;
                adjustmentState.timeout_at_milliseconds = 0;
                adjustmentState.ready = true;
            } else {
                continuos_adjustment_state_t& adjustmentState = _continuos_adjustments[static_cast<size_t>(_continuos_adjustment_count++)];
                adjustmentState.adjustment_range_index = ii;
                adjustmentState.last_rc_data = 0;
            }
        }
        ++ii;
    }
}

void RcAdjustments::process_stepwise_adjustments(const ReceiverBase& receiver, FlightController& flight_controller, Blackbox* blackbox, rates_t& rates, bool canUseRxData)
{
    enum { FREQUENCY_2HZ_IN_MILLISECONDS = 1000 / 2 };
    const time_ms32_t timeNowMs = time_ms();

    for (timed_adjustment_state_t& adjustmentState : _stepwise_adjustments) {
        const rc_adjustment_range_t& adjustment_range = _adjustment_ranges[adjustmentState.adjustment_range_index];
        const adjustment_config_t& adjustment_config = (*_default_adjustment_configs)[adjustment_range.adjustment_config - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
        const adjustment_e adjustment = adjustment_config.adjustment;

        if (!receiver.is_range_active(adjustment_range.aux_channel_index, adjustment_range.range_start, adjustment_range.range_end) || adjustment == ADJUSTMENT_NONE) {
            adjustmentState.timeout_at_milliseconds = 0;
            continue;
        }
        if (time_difference_ms(timeNowMs, adjustmentState.timeout_at_milliseconds) >= 0) { // cppcheck-suppress knownConditionTrueFalse

            adjustmentState.timeout_at_milliseconds = timeNowMs + FREQUENCY_2HZ_IN_MILLISECONDS;
            adjustmentState.ready = true;
        }
        if (!canUseRxData) {
            continue;
        }
        if (adjustment_config.mode == ADJUSTMENT_MODE_STEP) {
            int32_t delta {};
            const uint16_t aux_channel_pwm = receiver.get_auxiliary_channel(adjustment_range.aux_switch_channel_index);
            if (aux_channel_pwm > ReceiverBase::CHANNEL_MIDDLE + 200) {
                delta = adjustment_config.data.step;
            } else if (aux_channel_pwm < ReceiverBase::CHANNEL_MIDDLE - 200) {
                delta = -adjustment_config.data.step;
            } else {
                // returning the switch to the middle immediately resets the ready state
                adjustmentState.ready = true;
                adjustmentState.timeout_at_milliseconds = timeNowMs + FREQUENCY_2HZ_IN_MILLISECONDS;
                continue;
            }
            if (!adjustmentState.ready) {
                continue;
            }
            [[maybe_unused]]const int newValue = apply_step_adjustment(flight_controller, blackbox, rates, adjustment, delta);
            adjustmentState.ready = false;

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
            updateOsdAdjustmentData(newValue, adjustment_config->adjustment);
#endif
        }
    }
}

void RcAdjustments::process_continuous_adjustments(const ReceiverBase& receiver, FlightController& flight_controller, Cockpit& cockpit, Blackbox* blackbox, OSD* osd)
{
    for (continuos_adjustment_state_t& adjustmentState : _continuos_adjustments) {
        const rc_adjustment_range_t& adjustment_range = _adjustment_ranges[adjustmentState.adjustment_range_index];
        //const uint8_t channel_index = NON_AUX_CHANNEL_COUNT + adjustment_range->aux_switch_channel_index;
        const adjustment_config_t& adjustment_config = (*_default_adjustment_configs)[adjustment_range.adjustment_config - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
        const adjustment_e adjustment = adjustment_config.adjustment;

        if (receiver.is_range_active(adjustment_range.aux_channel_index, adjustment_range.range_start, adjustment_range.range_end) && adjustment != ADJUSTMENT_NONE) {

            const uint16_t aux_channel_pwm = receiver.get_auxiliary_channel(adjustment_range.aux_switch_channel_index);
            if (aux_channel_pwm != adjustmentState.last_rc_data) {
                [[maybe_unused]] int newValue = -1;

                if (adjustment_config.mode == ADJUSTMENT_MODE_SELECT) {
                    const uint8_t switch_positions = adjustment_config.data.switch_positions;
                    //if (adjustment == ADJUSTMENT_RATE_PROFILE && systemConfig()->rateProfile6PosSwitch) {
                    //    switch_positions =  6;
                    //}
                    const int32_t range_width = (2100 - 900) / switch_positions;
                    const uint8_t position = static_cast<uint8_t>((std::clamp(aux_channel_pwm, uint16_t{900}, uint16_t{2100 - 1}) - 900) / range_width);
                    newValue = apply_select_adjustment(flight_controller, cockpit, blackbox, osd, adjustment, position);
                } else {
                    // If setting is defined for step adjustment and center value has been specified, apply values directly (scaled) from aux channel
                    if (adjustment_range.adjustmentCenter && (adjustment_config.mode == ADJUSTMENT_MODE_STEP)) {
                        const int value = (((aux_channel_pwm - ReceiverBase::CHANNEL_MIDDLE) * adjustment_range.adjustment_scale) / (ReceiverBase::CHANNEL_MIDDLE - ReceiverBase::CHANNEL_LOW)) + adjustment_range.adjustmentCenter;
                        newValue = apply_absolute_adjustment(flight_controller, blackbox, cockpit.get_rates(), adjustment, value);
                    }
                }
#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
                updateOsdAdjustmentData(newValue, adjustment_config->adjustment);
#endif
                adjustmentState.last_rc_data = static_cast<int16_t>(aux_channel_pwm);
            }
        } else {
            adjustmentState.last_rc_data = 0;
        }
    }
}

void RcAdjustments::process_adjustments(const ReceiverBase& receiver, FlightController& flight_controller, Blackbox* blackbox, Cockpit& cockpit, OSD* osd, bool isReceiverSignal)
{
    // Recalculate the new active adjustments if required
    if (_stepwise_adjustment_count == ADJUSTMENT_RANGE_COUNT_INVALID) {
        // This can take up to 30us and is only called when not armed so it doesn't impact flight
        calculate_active_adjustment_ranges();
    }
    process_stepwise_adjustments(receiver, flight_controller, blackbox, cockpit.get_rates(), isReceiverSignal);

    if (isReceiverSignal) {
        process_continuous_adjustments(receiver, flight_controller, cockpit, blackbox, osd);
    }
}
