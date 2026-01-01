#include "Cockpit.h"
#include "FlightController.h"
#include "RC_Adjustments.h"
#include "RC_Modes.h"
#include "ReceiverBase.h"
#if defined (USE_OSD_PROFILES)
#include "OSD.h"
#endif
#include "Rates.h"

#include <Blackbox.h>
#include <TimeMicroseconds.h>

#include <cassert>
#include <cstring>


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

void RC_Adjustments::setAdjustmentRanges(const adjustment_ranges_t& adjustmentRanges)
{
    _adjustmentRanges = adjustmentRanges;
}

const RC_Adjustments::adjustment_ranges_t& RC_Adjustments::getAdjustmentRanges() const
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

const RC_Adjustments::adjustment_range_t& RC_Adjustments::getAdjustmentRange(size_t index) const
{
    assert(index < MAX_ADJUSTMENT_RANGE_COUNT && "Adjustment Range Index too large");
    return _adjustmentRanges[index];
}

void RC_Adjustments::setAdjustmentRange(size_t index, const adjustment_range_t& adjustmentRange)
{
    if (index < MAX_ADJUSTMENT_RANGE_COUNT) {
        _adjustmentRanges[index] = adjustmentRange;
    }
}

void RC_Adjustments::activeAdjustmentRangeReset()
{
    _stepwiseAdjustmentCount = ADJUSTMENT_RANGE_COUNT_INVALID;
}

void RC_Adjustments::blackboxLogInflightAdjustmentEvent(adjustment_e adjustment, int32_t newValue)
{
#if defined(USE_BLACKBOX)
    if (_blackbox) {
        Blackbox::log_event_data_u logEvent {
            .inflightAdjustment = {
                .newValue = newValue,
                .newFloatValue =  0.0F,
                .adjustmentFunction = adjustment,
                .floatFlag = false
            }
        };
        _blackbox->logEvent(Blackbox::LOG_EVENT_INFLIGHT_ADJUSTMENT, &logEvent);
    }
#else
    (void)adjustment;
    (void)newValue;
#endif
}

int32_t RC_Adjustments::applyStepAdjustment(FlightController& flightController, rates_t& rates, adjustment_e adjustment, int32_t delta)
{
    beeperConfirmationBeeps(delta > 0 ? 2 : 1);

    switch (adjustment) {
    case ADJUSTMENT_RC_RATE:
        [[fallthrough]];
    case ADJUSTMENT_ROLL_RC_RATE: {
        //newValue = constrain((int)controlRateConfig->rcRates[FD_ROLL] + delta, 1, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rcRates[FlightController::FD_ROLL] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcRates[FlightController::FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_RATE, newValue);
        if (adjustment == ADJUSTMENT_ROLL_RC_RATE) {
            return newValue;
        }
        [[fallthrough]]; // for combined ADJUSTMENT_RC_EXPO
    }
    case ADJUSTMENT_PITCH_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rcRates[FlightController::FD_PITCH] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcRates[FlightController::FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_P:
    case ADJUSTMENT_PITCH_P: {
        const FlightController::PIDF_uint16_t pid = flightController.getPID_MSP(FlightController::PITCH_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kp + delta), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.setPID_P_MSP(FlightController::PITCH_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_P, newValue);
        if (adjustment == ADJUSTMENT_PITCH_P) {
            return newValue;
        }
        [[fallthrough]]; // for combined ADJUSTMENT_PITCH_ROLL_P
    }
    case ADJUSTMENT_ROLL_P: {
        const FlightController::PIDF_uint16_t pid = flightController.getPID_MSP(FlightController::ROLL_RATE_DPS);
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(pid.kp + delta), uint16_t{0}, FlightController::PID_GAIN_MAX); // FIXME magic numbers repeated in cli.c
        flightController.setPID_P_MSP(FlightController::ROLL_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_P, newValue);
        return newValue;
    }
    default:
        return -1;
    };
}

int32_t RC_Adjustments::applyAbsoluteAdjustment(FlightController& flightController, rates_t& rates, adjustment_e adjustment, int32_t value)
{
    switch (adjustment) {
    case ADJUSTMENT_RC_RATE:
    case ADJUSTMENT_ROLL_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcRates[FlightController::FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_RATE, newValue);
        if (adjustment == ADJUSTMENT_ROLL_RC_RATE) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_PITCH_RC_RATE: {
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(value), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcRates[FlightController::FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_RATE, newValue);
        return newValue;
    }
    case ADJUSTMENT_PITCH_ROLL_P:
    case ADJUSTMENT_PITCH_P: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.setPID_P_MSP(FlightController::PITCH_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_P, newValue);
        if (adjustment == ADJUSTMENT_PITCH_P) {
            return newValue;
        }
        [[fallthrough]]; // for combined adjustment
    }
    case ADJUSTMENT_ROLL_P: {
        const uint16_t newValue = std::clamp(static_cast<uint16_t>(value), uint16_t{0}, FlightController::PID_GAIN_MAX);
        flightController.setPID_P_MSP(FlightController::ROLL_RATE_DPS, newValue);
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_P, newValue);
        return newValue;
    }
    default:
        return -1;
    };
}

uint8_t RC_Adjustments::applySelectAdjustment(FlightController& flightController, Cockpit& cockpit, OSD* osd, adjustment_e adjustment, uint8_t position)
{
    uint8_t beeps = 0;

    switch (adjustment) {
    case ADJUSTMENT_RATE_PROFILE:
        if (cockpit.getCurrentRateProfileIndex() != position) {
            cockpit.setCurrentRateProfileIndex(position);
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RATE_PROFILE, position);
            beeps = position + 1;
        }
        break;
    case ADJUSTMENT_HORIZON_STRENGTH: {
        const uint8_t newValue = std::clamp(position, uint8_t{0}, uint8_t{FlightController::PID_GAIN_MAX}); // FIXME magic numbers repeated in serial_cli.c
        const FlightController::PIDF_uint16_t pid = flightController.getPID_MSP(FlightController::ROLL_ANGLE_DEGREES);
        if (pid.kd != newValue) {
            beeps = static_cast<uint8_t>((newValue - pid.kd) / 8) + 1;
            flightController.setPID_P_MSP(FlightController::ROLL_ANGLE_DEGREES, newValue);
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_HORIZON_STRENGTH, position);
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

void RC_Adjustments::calcActiveAdjustmentRanges()
{
    adjustment_range_t defaultAdjustmentRange {};

    _stepwiseAdjustmentCount = 0;
    _continuosAdjustmentCount = 0;
    uint8_t ii = 0;
    for (adjustment_range_t& adjustmentRange: _adjustmentRanges) {
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

void RC_Adjustments::setConfigDirty()
{
}

void RC_Adjustments::setConfigDirtyIfNotPermanent(const ReceiverBase::channel_range_t& range)
{
    if (!(range.startStep == ReceiverBase::RANGE_STEP_MIN && range.endStep == ReceiverBase::RANGE_STEP_MAX)) {
        // Only set the configuration dirty if this range is NOT permanently enabled (and the config thus never used).
        setConfigDirty();
    }
}

void RC_Adjustments::processStepwiseAdjustments(const ReceiverBase& receiver, FlightController& flightController, rates_t& rates, bool canUseRxData)
{
    enum { FREQUENCY_2HZ_IN_MILLISECONDS = 1000 / 2 };
    const timeMs32_t timeNowMs = timeMs();

    for (timed_adjustment_state_t& adjustmentState : _stepwiseAdjustments) {
        const adjustment_range_t& adjustmentRange = _adjustmentRanges[adjustmentState.adjustmentRangeIndex];
        const adjustment_config_t& adjustmentConfig = (*_defaultAdjustmentConfigs)[adjustmentRange.adjustmentConfig - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
        const adjustment_e adjustment = adjustmentConfig.adjustment;

        if (!receiver.isRangeActive(adjustmentRange.auxChannelIndex, adjustmentRange.range) || adjustment == ADJUSTMENT_NONE) {
            adjustmentState.timeoutAtMilliseconds = 0;
            continue;
        }

        if (timeDifferenceMs(timeNowMs, adjustmentState.timeoutAtMilliseconds) >= 0) { // cppcheck-suppress knownConditionTrueFalse

            adjustmentState.timeoutAtMilliseconds = timeNowMs + FREQUENCY_2HZ_IN_MILLISECONDS;
            adjustmentState.ready = true;
        }

        if (!canUseRxData) {
            continue;
        }

        if (adjustmentConfig.mode == ADJUSTMENT_MODE_STEP) {
            int32_t delta {};
            const uint16_t auxChannelPWM = receiver.getAuxiliaryChannel(adjustmentRange.auxSwitchChannelIndex);
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

            [[maybe_unused]]const int newValue = applyStepAdjustment(flightController, rates, adjustment, delta);

            setConfigDirty();
            //!!pidInitConfig(currentPidProfile);

            adjustmentState.ready = false;

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
            updateOsdAdjustmentData(newValue, adjustmentConfig->adjustmentFunction);
#endif
        }
    }
}

void RC_Adjustments::processContinuosAdjustments(const ReceiverBase& receiver, FlightController& flightController, Cockpit& cockpit, OSD* osd)
{
    for (continuos_adjustment_state_t& adjustmentState : _continuosAdjustments) {
        const adjustment_range_t& adjustmentRange = _adjustmentRanges[adjustmentState.adjustmentRangeIndex];
        //const uint8_t channelIndex = NON_AUX_CHANNEL_COUNT + adjustmentRange->auxSwitchChannelIndex;
        const adjustment_config_t& adjustmentConfig = (*_defaultAdjustmentConfigs)[adjustmentRange.adjustmentConfig - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
        const adjustment_e adjustment = adjustmentConfig.adjustment;

        if (receiver.isRangeActive(adjustmentRange.auxChannelIndex, adjustmentRange.range) && adjustment != ADJUSTMENT_NONE) {

            const uint16_t auxChannelPWM = receiver.getAuxiliaryChannel(adjustmentRange.auxSwitchChannelIndex);
            if (auxChannelPWM != adjustmentState.lastRcData) {
                [[maybe_unused]] int newValue = -1;

                if (adjustmentConfig.mode == ADJUSTMENT_MODE_SELECT) {
                    const uint8_t switchPositions = adjustmentConfig.data.switchPositions;
                    //if (adjustment == ADJUSTMENT_RATE_PROFILE && systemConfig()->rateProfile6PosSwitch) {
                    //    switchPositions =  6;
                    //}
                    const int32_t rangeWidth = (2100 - 900) / switchPositions;
                    const uint8_t position = static_cast<uint8_t>((std::clamp(auxChannelPWM, uint16_t{900}, uint16_t{2100 - 1}) - 900) / rangeWidth);
                    newValue = applySelectAdjustment(flightController, cockpit, osd, adjustment, position);

                    setConfigDirtyIfNotPermanent(adjustmentRange.range);
                } else {
                    // If setting is defined for step adjustment and center value has been specified, apply values directly (scaled) from aux channel
                    if (adjustmentRange.adjustmentCenter && (adjustmentConfig.mode == ADJUSTMENT_MODE_STEP)) {
                        const int value = (((auxChannelPWM - ReceiverBase::CHANNEL_MIDDLE) * adjustmentRange.adjustmentScale) / (ReceiverBase::CHANNEL_MIDDLE - ReceiverBase::CHANNEL_LOW)) + adjustmentRange.adjustmentCenter;

                        newValue = applyAbsoluteAdjustment(flightController, cockpit.getRates(), adjustment, value);

                        setConfigDirtyIfNotPermanent(adjustmentRange.range);
                        //!!pidInitConfig(currentPidProfile);
                    }
                }
#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
                updateOsdAdjustmentData(newValue, adjustmentConfig->adjustmentFunction);
#endif
                adjustmentState.lastRcData = auxChannelPWM;
            }
        } else {
            adjustmentState.lastRcData = 0;
        }
    }
}

void RC_Adjustments::processAdjustments(const ReceiverBase& receiver, FlightController& flightController, Cockpit& cockpit, OSD* osd, bool isReceiverSignal)
{
    // Recalculate the new active adjustments if required
    if (_stepwiseAdjustmentCount == ADJUSTMENT_RANGE_COUNT_INVALID) {
        // This can take up to 30us and is only call when not armed so ignore this timing as it doesn't impact flight
        //schedulerIgnoreTaskExecTime();
        calcActiveAdjustmentRanges();
    }
    processStepwiseAdjustments(receiver, flightController, cockpit.getRates(), isReceiverSignal);

    if (isReceiverSignal) {
        processContinuosAdjustments(receiver, flightController, cockpit, osd);
    }
}
