#include "Cockpit.h"
#include "FlightController.h"
#include "RC_Adjustments.h"
#if defined (USE_OSD_PROFILES)
#include "OSD.h"
#endif
#include "Rates.h"

#include <blackbox.h>
#include <cassert>
#include <cstring>
#include <receiver_base.h>
#include <time_microseconds.h>


RC_Adjustments::RC_Adjustments(const adjustment_configs_t* defaultAdjustmentConfigs) :
    _defaultAdjustmentConfigs(defaultAdjustmentConfigs)
{
#if !defined(FRAMEWORK_TEST)
    assert(defaultAdjustmentConfigs != nullptr);
#endif
}

void RC_Adjustments::beeperConfirmationBeeps(uint8_t beepCount)
{
    (void)beepCount;
}

void RC_Adjustments::setAdjustmentRanges(const rc_adjustment_ranges_t& adjustmentRanges)
{
    _adjustmentRanges = adjustmentRanges;
}

const rc_adjustment_ranges_t& RC_Adjustments::getAdjustmentRanges() const
{
    return _adjustmentRanges;
}

void RC_Adjustments::setAdjustmentConfigs(const adjustment_configs_t& adjustmentConfigs)
{
    _adjustmentConfigs = adjustmentConfigs;
}

const RC_Adjustments::adjustment_configs_t& RC_Adjustments::getAdjustmentConfigs() const
{
    return _adjustmentConfigs;
}

const rc_adjustment_range_t& RC_Adjustments::getAdjustmentRange(size_t index) const
{
    assert(index < RC_MAX_ADJUSTMENT_RANGE_COUNT && "Adjustment Range Index too large");
    return _adjustmentRanges[index];
}

void RC_Adjustments::setAdjustmentRange(size_t index, const rc_adjustment_range_t& adjustmentRange)
{
    if (index < RC_MAX_ADJUSTMENT_RANGE_COUNT) {
        _adjustmentRanges[index] = adjustmentRange;
    }
}

void RC_Adjustments::activeAdjustmentRangeReset()
{
    _stepwiseAdjustmentCount = ADJUSTMENT_RANGE_COUNT_INVALID;
}

void RC_Adjustments::blackboxLogInflightAdjustmentEvent(Blackbox* blackbox, adjustment_e adjustment, int32_t newValue)
{
#if defined(USE_BLACKBOX)
    if (blackbox) {
        Blackbox::log_event_data_u logEvent {
            .inflightAdjustment = {
                .newValue = newValue,
                .newFloatValue =  0.0F,
                .adjustment = adjustment,
                .floatFlag = false
            }
        };
        blackbox->logEvent(Blackbox::LOG_EVENT_INFLIGHT_ADJUSTMENT, &logEvent);
    }
#else
    (void)blackbox;
    (void)adjustment;
    (void)newValue;
#endif
}

int32_t RC_Adjustments::applyStepAdjustment(FlightController& flightController, Blackbox* blackbox, rates_t& rates, adjustment_e adjustment, int32_t delta)
{
    beeperConfirmationBeeps(delta > 0 ? 2 : 1);

    switch (adjustment) {
    case ADJUSTMENT_RC_RATE:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_ROLL_RC_RATE: {
        //newValue = constrain((int)controlRateConfig->rcRates[FD_ROLL] + delta, 1, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rcRates[FlightController::FD_ROLL] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcRates[FlightController::FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_RC_RATE, newValue);
        if (adjustment == ADJUSTMENT_ROLL_RC_RATE) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_PITCH_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rcRates[FlightController::FD_PITCH] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcRates[FlightController::FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_RC_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_RC_EXPO:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_ROLL_RC_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rcExpos[FlightController::FD_ROLL] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcExpos[FlightController::FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_RC_EXPO, newValue);
        if (adjustment == ADJUSTMENT_ROLL_RC_EXPO) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_PITCH_RC_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rcExpos[FlightController::FD_PITCH] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcExpos[FlightController::FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_RC_EXPO, newValue);
        return newValue;
    }
    case ADJUSTMENT_THROTTLE_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.throttleExpo + delta), uint8_t{0}, uint8_t{rates_t::THROTTLE_MAX});
        rates.throttleExpo = newValue;
        //!!initRcProcessing();
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_THROTTLE_EXPO, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_RATE:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rates[FlightController::FD_PITCH] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[FlightController::FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_RATE, newValue);
        if (adjustment == ADJUSTMENT_PITCH_RATE) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rates[FlightController::FD_ROLL] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[FlightController::FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rates[FlightController::FD_YAW] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[FlightController::FD_YAW] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_YAW_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_P:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_P: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::PITCH_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kp + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_p_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_P, newValue);
        if (adjustment == ADJUSTMENT_PITCH_P) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_P: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::ROLL_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kp + delta), uint16_t{0}, FlightController::PID_GAIN_MAX); // FIXME magic numbers repeated in cli.c
        flightController.set_pid_p_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_P, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_I:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_I: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::PITCH_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.ki + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_i_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_I, newValue);
        if (adjustment == ADJUSTMENT_PITCH_I) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_I: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::ROLL_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.ki + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_i_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_I, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_D:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_D: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::PITCH_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kd + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_d_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_D, newValue);
        if (adjustment == ADJUSTMENT_PITCH_D) {
            return newValue;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_D
        [[fallthrough]];
    }
    case ADJUSTMENT_ROLL_D: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::ROLL_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kd + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_d_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_D, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_P: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::YAW_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kp + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_p_msp(FlightController::YAW_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_YAW_P, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_I: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::YAW_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.ki + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_i_msp(FlightController::YAW_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_YAW_I, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_D:
        return -1;
    case ADJUSTMENT_PITCH_ROLL_K:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_K: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::PITCH_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kk + delta), uint16_t{0}, FlightController::K_GAIN_MAX);
        flightController.set_pid_k_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_K, newValue);
        if (adjustment == ADJUSTMENT_PITCH_K) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_K: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::ROLL_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kk + delta), uint16_t{0}, FlightController::K_GAIN_MAX);
        flightController.set_pid_k_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_K, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_K: {
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::YAW_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kk + delta), uint16_t{0}, FlightController::K_GAIN_MAX);
        flightController.set_pid_k_msp(FlightController::YAW_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_YAW_K, newValue);
        return newValue;
    }
#if defined(USE_FEEDFORWARD)
    case ADJUSTMENT_FEEDFORWARD_TRANSITION: {
        newValue = constrain(current_pidProfile->feedforward_transition + delta, 1, 100); // FIXME magic numbers repeated in cli.c
        current_pidProfile->feedforward_transition = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_FEEDFORWARD_TRANSITION, newValue);
        return newValue;
    }
#endif
    default:
        return -1;
    };
}

int32_t RC_Adjustments::applyAbsoluteAdjustment(FlightController& flightController, Blackbox* blackbox, rates_t& rates, adjustment_e adjustment, int32_t value)
{
    switch (adjustment) {
    case ADJUSTMENT_RC_RATE:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_ROLL_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcRates[FlightController::FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_RC_RATE, newValue);
        if (adjustment == ADJUSTMENT_ROLL_RC_RATE) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_PITCH_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcRates[FlightController::FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_RC_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcRates[FlightController::FD_YAW] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_YAW_RC_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_RC_EXPO:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_ROLL_RC_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_EXPOS_MAX});
        rates.rcExpos[rates_t::ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_RC_EXPO, newValue);
        if (adjustment == ADJUSTMENT_ROLL_RC_EXPO) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_PITCH_RC_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_EXPOS_MAX});
        rates.rcExpos[rates_t::PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_RC_EXPO, newValue);
        return newValue;
    }
    case ADJUSTMENT_THROTTLE_EXPO: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{0}, uint8_t{rates_t::RC_EXPOS_MAX});
        rates.throttleExpo = newValue;
        //initRcProcessing();
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_THROTTLE_EXPO, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_RATE:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{0}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[rates_t::PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_RATE, newValue);
        if (adjustment == ADJUSTMENT_PITCH_RATE) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{0}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[rates_t::ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{0}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rates[rates_t::YAW] = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_YAW_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_P:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_P: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_p_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_P, newValue);
        if (adjustment == ADJUSTMENT_PITCH_P) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_P: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_p_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_P, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_I:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_I: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_i_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_I, newValue);
        if (adjustment == ADJUSTMENT_PITCH_I) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_I: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_i_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_I, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_D:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_D: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_d_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_D, newValue);
        if (adjustment == ADJUSTMENT_PITCH_D) {
            return newValue;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_D
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_D: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_d_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_D, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_P: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_p_msp(FlightController::YAW_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_YAW_P, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_I: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.set_pid_i_msp(FlightController::YAW_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_YAW_I, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_D:
        return -1;
    case ADJUSTMENT_PITCH_ROLL_K:
        [[fallthrough]]; // for combined adjustment
    case ADJUSTMENT_PITCH_K: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::K_GAIN_MAX);
        flightController.set_pid_k_msp(FlightController::PITCH_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_PITCH_K, newValue);
        if (adjustment == ADJUSTMENT_PITCH_K) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_K: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::K_GAIN_MAX);
        flightController.set_pid_k_msp(FlightController::ROLL_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_ROLL_K, newValue);
        return newValue;
    }
    case ADJUSTMENT_YAW_K: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::K_GAIN_MAX);
        flightController.set_pid_k_msp(FlightController::YAW_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_YAW_K, newValue);
        return newValue;
    }
#if defined(USE_FEEDFORWARD)
    case ADJUSTMENT_FEEDFORWARD_TRANSITION: {
        newValue = constrain(value, 1, 100); // FIXME magic numbers repeated in cli.c
        current_pidProfile->feedforward_transition = newValue;
        blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_FEEDFORWARD_TRANSITION, newValue);
        return newValue;
    }
#endif
    default:
        return -1;
    };
}

uint8_t RC_Adjustments::applySelectAdjustment(FlightController& flightController, Cockpit& cockpit, Blackbox* blackbox, [[maybe_unused]] OSD* osd, adjustment_e adjustment, uint8_t position)
{
    (void)cockpit;
    uint8_t beeps = 0;

    switch (adjustment) {
    /*!!case ADJUSTMENT_RATE_PROFILE:
        if (cockpit.get_current_rate_profile_index() != position) {
            cockpit.set_current_rate_profile_index(position);
            blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_RATE_PROFILE, position);
            beeps = position + 1;
        }
        break;*/
    case ADJUSTMENT_HORIZON_STRENGTH: {
        const uint8_t newValue = std::clamp(position, uint8_t{0}, uint8_t{FlightController::PID_GAIN_MAX}); // FIXME magic numbers repeated in serial_cli.c
        const FlightController::PIDF_uint16_t pid = flightController.get_pid_msp(FlightController::ROLL_ANGLE_DEGREES);
        if (pid.kd != newValue) {
            beeps = static_cast<uint8_t>((newValue - pid.kd) / 8) + 1;
            flightController.set_pid_p_msp(FlightController::ROLL_ANGLE_DEGREES, newValue);
            blackboxLogInflightAdjustmentEvent(blackbox, ADJUSTMENT_HORIZON_STRENGTH, position);
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
        if (osd != nullptr && osd->getProfile() != (position + 1)) {
            osd->setProfile(position + 1);
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
        beeperConfirmationBeeps(beeps);
    }
    return position;
}

void RC_Adjustments::calculateActiveAdjustmentRanges()
{
    rc_adjustment_range_t defaultAdjustmentRange {};

    _stepwiseAdjustmentCount = 0;
    _continuosAdjustmentCount = 0;
    uint8_t ii = 0;
    for (rc_adjustment_range_t& adjustmentRange: _adjustmentRanges) {
        if (memcmp(&adjustmentRange, &defaultAdjustmentRange, sizeof(defaultAdjustmentRange)) != 0) {
            const adjustment_config_t& adjustmentConfig = (*_defaultAdjustmentConfigs)[adjustmentRange.adjustmentConfig - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
            if (adjustmentRange.adjustmentCenter == 0 && adjustmentConfig.mode != ADJUSTMENT_MODE_SELECT) {
                timed_adjustment_state_t& adjustmentState = _stepwiseAdjustments[static_cast<size_t>(_stepwiseAdjustmentCount++)];
                adjustmentState.adjustmentRangeIndex = ii;
                adjustmentState.timeoutAtMilliseconds = 0;
                adjustmentState.ready = true;
            } else {
                continuos_adjustment_state_t& adjustmentState = _continuosAdjustments[static_cast<size_t>(_continuosAdjustmentCount++)];
                adjustmentState.adjustmentRangeIndex = ii;
                adjustmentState.lastRcData = 0;
            }
        }
        ++ii;
    }
}

void RC_Adjustments::processStepwiseAdjustments(const ReceiverBase& receiver, FlightController& flightController, Blackbox* blackbox, rates_t& rates, bool canUseRxData)
{
    enum { FREQUENCY_2HZ_IN_MILLISECONDS = 1000 / 2 };
    const time_ms32_t timeNowMs = time_ms();

    for (timed_adjustment_state_t& adjustmentState : _stepwiseAdjustments) {
        const rc_adjustment_range_t& adjustmentRange = _adjustmentRanges[adjustmentState.adjustmentRangeIndex];
        const adjustment_config_t& adjustmentConfig = (*_defaultAdjustmentConfigs)[adjustmentRange.adjustmentConfig - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
        const adjustment_e adjustment = adjustmentConfig.adjustment;

        if (!receiver.is_range_active(adjustmentRange.aux_channel_index, adjustmentRange.range_start, adjustmentRange.range_end) || adjustment == ADJUSTMENT_NONE) {
            adjustmentState.timeoutAtMilliseconds = 0;
            continue;
        }
        if (time_difference_ms(timeNowMs, adjustmentState.timeoutAtMilliseconds) >= 0) { // cppcheck-suppress knownConditionTrueFalse

            adjustmentState.timeoutAtMilliseconds = timeNowMs + FREQUENCY_2HZ_IN_MILLISECONDS;
            adjustmentState.ready = true;
        }
        if (!canUseRxData) {
            continue;
        }
        if (adjustmentConfig.mode == ADJUSTMENT_MODE_STEP) {
            int32_t delta {};
            const uint16_t auxChannelPWM = receiver.get_auxiliary_channel(adjustmentRange.auxSwitchChannelIndex);
            if (auxChannelPWM > ReceiverBase::CHANNEL_MIDDLE + 200) {
                delta = adjustmentConfig.data.step;
            } else if (auxChannelPWM < ReceiverBase::CHANNEL_MIDDLE - 200) {
                delta = -adjustmentConfig.data.step;
            } else {
                // returning the switch to the middle immediately resets the ready state
                adjustmentState.ready = true;
                adjustmentState.timeoutAtMilliseconds = timeNowMs + FREQUENCY_2HZ_IN_MILLISECONDS;
                continue;
            }
            if (!adjustmentState.ready) {
                continue;
            }
            [[maybe_unused]]const int newValue = applyStepAdjustment(flightController, blackbox, rates, adjustment, delta);
            adjustmentState.ready = false;

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
            updateOsdAdjustmentData(newValue, adjustmentConfig->adjustment);
#endif
        }
    }
}

void RC_Adjustments::processContinuosAdjustments(const ReceiverBase& receiver, FlightController& flightController, Cockpit& cockpit, Blackbox* blackbox, OSD* osd)
{
    for (continuos_adjustment_state_t& adjustmentState : _continuosAdjustments) {
        const rc_adjustment_range_t& adjustmentRange = _adjustmentRanges[adjustmentState.adjustmentRangeIndex];
        //const uint8_t channelIndex = NON_AUX_CHANNEL_COUNT + adjustmentRange->auxSwitchChannelIndex;
        const adjustment_config_t& adjustmentConfig = (*_defaultAdjustmentConfigs)[adjustmentRange.adjustmentConfig - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
        const adjustment_e adjustment = adjustmentConfig.adjustment;

        if (receiver.is_range_active(adjustmentRange.aux_channel_index, adjustmentRange.range_start, adjustmentRange.range_end) && adjustment != ADJUSTMENT_NONE) {

            const uint16_t auxChannelPWM = receiver.get_auxiliary_channel(adjustmentRange.auxSwitchChannelIndex);
            if (auxChannelPWM != adjustmentState.lastRcData) {
                [[maybe_unused]] int newValue = -1;

                if (adjustmentConfig.mode == ADJUSTMENT_MODE_SELECT) {
                    const uint8_t switchPositions = adjustmentConfig.data.switchPositions;
                    //if (adjustment == ADJUSTMENT_RATE_PROFILE && systemConfig()->rateProfile6PosSwitch) {
                    //    switchPositions =  6;
                    //}
                    const int32_t rangeWidth = (2100 - 900) / switchPositions;
                    const uint8_t position = static_cast<uint8_t>((std::clamp(auxChannelPWM, uint16_t{900}, uint16_t{2100 - 1}) - 900) / rangeWidth);
                    newValue = applySelectAdjustment(flightController, cockpit, blackbox, osd, adjustment, position);
                } else {
                    // If setting is defined for step adjustment and center value has been specified, apply values directly (scaled) from aux channel
                    if (adjustmentRange.adjustmentCenter && (adjustmentConfig.mode == ADJUSTMENT_MODE_STEP)) {
                        const int value = (((auxChannelPWM - ReceiverBase::CHANNEL_MIDDLE) * adjustmentRange.adjustmentScale) / (ReceiverBase::CHANNEL_MIDDLE - ReceiverBase::CHANNEL_LOW)) + adjustmentRange.adjustmentCenter;
                        newValue = applyAbsoluteAdjustment(flightController, blackbox, cockpit.getRates(), adjustment, value);
                    }
                }
#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
                updateOsdAdjustmentData(newValue, adjustmentConfig->adjustment);
#endif
                adjustmentState.lastRcData = static_cast<int16_t>(auxChannelPWM);
            }
        } else {
            adjustmentState.lastRcData = 0;
        }
    }
}

void RC_Adjustments::processAdjustments(const ReceiverBase& receiver, FlightController& flightController, Blackbox* blackbox, Cockpit& cockpit, OSD* osd, bool isReceiverSignal)
{
    // Recalculate the new active adjustments if required
    if (_stepwiseAdjustmentCount == ADJUSTMENT_RANGE_COUNT_INVALID) {
        // This can take up to 30us and is only called when not armed so it doesn't impact flight
        calculateActiveAdjustmentRanges();
    }
    processStepwiseAdjustments(receiver, flightController, blackbox, cockpit.getRates(), isReceiverSignal);

    if (isReceiverSignal) {
        processContinuosAdjustments(receiver, flightController, cockpit, blackbox, osd);
    }
}
