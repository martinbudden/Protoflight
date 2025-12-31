#include "FlightController.h"
#include "RC_Adjustments.h"
#include "Rates.h"

#include <Blackbox.h>

#include <cassert>
#include <cstring>


RC_Adjustments::RC_Adjustments(const adjustment_configs_t* defaultAdjustmentConfigs) : 
    _defaultAdjustmentConfigs(defaultAdjustmentConfigs)
{
#if !defined(FRAMEWORK_TEST)
    assert(defaultAdjustmentConfigs != nullptr);
#endif
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

void RC_Adjustments::blackboxLogInflightAdjustmentEvent(adjustment_function_e adjustmentFunction, int32_t newValue)
{
#if defined(USE_BLACKBOX)
    if (_blackbox) {
        Blackbox::log_event_data_u logEvent {
            .inflightAdjustment = {
                .newValue = newValue,
                .newFloatValue =  0.0F,
                .adjustmentFunction = adjustmentFunction,
                .floatFlag = false
            }
        };
        _blackbox->logEvent(Blackbox::LOG_EVENT_INFLIGHT_ADJUSTMENT, &logEvent);
    }
#else
    (void)adjustmentFunction;
    (void)newValue;
#endif
}

int32_t RC_Adjustments::applyStepAdjustment(rates_t& rates, FlightController& flightController, adjustment_function_e adjustmentFunction, int32_t delta)
{
    //beeperConfirmationBeeps(delta > 0 ? 2 : 1);

    switch (adjustmentFunction) {
    case ADJUSTMENT_RC_RATE:
        [[fallthrough]];
    case ADJUSTMENT_ROLL_RC_RATE: {
        //newValue = constrain((int)controlRateConfig->rcRates[FD_ROLL] + delta, 1, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        const uint8_t newValue = std::clamp(static_cast<uint8_t>(rates.rcRates[FlightController::FD_ROLL] + delta), uint8_t{1}, uint8_t{rates_t::RC_RATES_MAX});
        rates.rcRates[FlightController::FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_RATE, newValue);
        if (adjustmentFunction == ADJUSTMENT_ROLL_RC_RATE) {
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
        if (adjustmentFunction == ADJUSTMENT_PITCH_P) {
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

void RC_Adjustments::calcActiveAdjustmentRanges()
{
    enum { ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET = 1};
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
                adjustmentState.timeoutAt = 0;
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

void RC_Adjustments::processStepwiseAdjustments(rates_t& rates, FlightController& flightController, bool isReceiverSignal)
{
    (void)rates;
    (void)flightController;
    (void)isReceiverSignal;
}

void RC_Adjustments::processContinuosAdjustments(rates_t& rates, FlightController& flightController)
{
    (void)rates;
    (void)flightController;
}

void RC_Adjustments::processAdjustments(rates_t& rates, FlightController& flightController, bool isReceiverSignal)
{
    // Recalculate the new active adjustments if required
    if (_stepwiseAdjustmentCount == ADJUSTMENT_RANGE_COUNT_INVALID) {
        // This can take up to 30us and is only call when not armed so ignore this timing as it doesn't impact flight
        //schedulerIgnoreTaskExecTime();
        calcActiveAdjustmentRanges();
    }
    processStepwiseAdjustments(rates, flightController, isReceiverSignal);

    if (isReceiverSignal) {
        processContinuosAdjustments(rates, flightController);
    }
}

