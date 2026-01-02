#pragma once

#include <ReceiverBase.h>
#include <array>

class Blackbox;
class Cockpit;
class FlightController;
class OSD;
struct rates_t;


enum adjustment_e {
    ADJUSTMENT_NONE = 0,
    ADJUSTMENT_RC_RATE,
    ADJUSTMENT_RC_EXPO,
    ADJUSTMENT_THROTTLE_EXPO,
    ADJUSTMENT_PITCH_ROLL_RATE,
    ADJUSTMENT_YAW_RATE,
    ADJUSTMENT_PITCH_ROLL_P,
    ADJUSTMENT_PITCH_ROLL_I,
    ADJUSTMENT_PITCH_ROLL_D,
    ADJUSTMENT_YAW_P,
    ADJUSTMENT_YAW_I,
    ADJUSTMENT_YAW_D,
    ADJUSTMENT_RATE_PROFILE,
    ADJUSTMENT_PITCH_RATE,
    ADJUSTMENT_ROLL_RATE,
    ADJUSTMENT_PITCH_P,
    ADJUSTMENT_PITCH_I,
    ADJUSTMENT_PITCH_D,
    ADJUSTMENT_ROLL_P,
    ADJUSTMENT_ROLL_I,
    ADJUSTMENT_ROLL_D,
    ADJUSTMENT_YAW_RC_RATE,
    ADJUSTMENT_PITCH_ROLL_K,
    ADJUSTMENT_FEEDFORWARD_TRANSITION,
    ADJUSTMENT_HORIZON_STRENGTH,
    ADJUSTMENT_ROLL_RC_RATE,
    ADJUSTMENT_PITCH_RC_RATE,
    ADJUSTMENT_ROLL_RC_EXPO,
    ADJUSTMENT_PITCH_RC_EXPO,
    ADJUSTMENT_PID_AUDIO,
    ADJUSTMENT_PITCH_K, // called ADJUSTMENT_PITCH_F in betaflight
    ADJUSTMENT_ROLL_K,
    ADJUSTMENT_YAW_K,
    ADJUSTMENT_OSD_PROFILE,
    ADJUSTMENT_LED_PROFILE,
    ADJUSTMENT_LED_DIMMER,
    ADJUSTMENT_FUNCTION_COUNT
};


class RC_Adjustments {
public:
    virtual ~RC_Adjustments() = default;
private:
    // RC_Adjustments is not copyable or moveable
    RC_Adjustments(const RC_Adjustments&) = delete;
    RC_Adjustments& operator=(const RC_Adjustments&) = delete;
    RC_Adjustments(RC_Adjustments&&) = delete;
    RC_Adjustments& operator=(RC_Adjustments&&) = delete;
public:
    enum { MAX_ADJUSTMENT_RANGE_COUNT = 30 };
    enum { ADJUSTMENT_RANGE_COUNT_INVALID = -1 };

    enum adjustment_mode_e { ADJUSTMENT_MODE_STEP, ADJUSTMENT_MODE_SELECT };
    struct adjustment_range_t {
        // when aux channel is in range...
        ReceiverBase::channel_range_t range;
        uint8_t auxChannelIndex;
        // ..then apply the adjustment function to the auxSwitchChannel ...
        uint8_t adjustmentConfig;
        uint8_t auxSwitchChannelIndex;
        uint8_t adjustmentCenter;
        uint16_t adjustmentScale;
    };
    struct timed_adjustment_state_t {
        uint32_t timeoutAtMilliseconds;
        uint8_t adjustmentRangeIndex;
        bool ready;
    };
    struct continuos_adjustment_state_t {
        uint8_t adjustmentRangeIndex;
        int16_t lastRcData;
    };
    union adjustment_data_u {
        uint8_t step;
        uint8_t switchPositions;
    };
    struct adjustment_config_t {
        adjustment_e adjustment;
        adjustment_mode_e mode;
        adjustment_data_u data;
    };
    typedef std::array<adjustment_range_t, MAX_ADJUSTMENT_RANGE_COUNT> adjustment_ranges_t;
    typedef std::array<adjustment_config_t, ADJUSTMENT_FUNCTION_COUNT> adjustment_configs_t;
public:
    explicit RC_Adjustments(const adjustment_configs_t* defaultAdjustmentConfigs);

    void setAdjustmentRanges(const adjustment_ranges_t& adjustmentRanges);
    const adjustment_ranges_t& getAdjustmentRanges() const;

    const adjustment_range_t& getAdjustmentRange(size_t index) const;
    void setAdjustmentRange(size_t index, const adjustment_range_t& adjustmentRange);
    void activeAdjustmentRangeReset();

    void setAdjustmentConfigs(const adjustment_configs_t& adjustmentConfigs);
    const adjustment_configs_t& getAdjustmentConfigs() const;

    void processAdjustments(const ReceiverBase& receiver, FlightController& flightController, Cockpit& cockpit, OSD* osd, bool isReceiverSignal);
private:
    enum { ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET = 1};
    void blackboxLogInflightAdjustmentEvent(adjustment_e adjustment, int32_t newValue);
    int32_t applyStepAdjustment(FlightController& flightController, rates_t& rates, adjustment_e adjustment, int32_t delta);
    int32_t applyAbsoluteAdjustment(FlightController& flightController, rates_t& rates, adjustment_e adjustment, int32_t value);
    uint8_t applySelectAdjustment(FlightController& flightController, Cockpit& cockpit, OSD* osd, adjustment_e adjustment, uint8_t position);
    void processStepwiseAdjustments(const ReceiverBase& receiver, FlightController& flightController, rates_t& rates, bool canUseRxData);
    void processContinuosAdjustments(const ReceiverBase& receiver, FlightController& flightController, Cockpit& cockpit, OSD* osd);
    void calcActiveAdjustmentRanges();
    void beeperConfirmationBeeps(uint8_t beepCount);

    const char *getRangeName();
    int getRangeValue();
private:
    Blackbox* _blackbox {nullptr};
    int32_t _stepwiseAdjustmentCount {ADJUSTMENT_RANGE_COUNT_INVALID};
    int32_t _continuosAdjustmentCount {};
    std::array<timed_adjustment_state_t, MAX_ADJUSTMENT_RANGE_COUNT> _stepwiseAdjustments {};
    std::array<continuos_adjustment_state_t, MAX_ADJUSTMENT_RANGE_COUNT> _continuosAdjustments {};
    std::array<adjustment_range_t, MAX_ADJUSTMENT_RANGE_COUNT> _adjustmentRanges {};
    std::array<adjustment_config_t, ADJUSTMENT_FUNCTION_COUNT> _adjustmentConfigs {};
    const adjustment_configs_t* _defaultAdjustmentConfigs; 
};
