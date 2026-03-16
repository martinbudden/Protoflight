#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

class Blackbox;
class Cockpit;
class FlightController;
class OSD;
class ReceiverBase;
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


struct rc_adjustment_range_t {
    // when aux channel is in range...
    uint8_t range_start;
    uint8_t range_end;
    uint8_t aux_channel_index;
    // ..then apply the adjustment function to the aux_switch_channel ...
    uint8_t adjustment_config;
    uint8_t aux_switch_channel_index;
    uint8_t adjustmentCenter;
    uint16_t adjustment_scale;
};

static constexpr size_t RC_MAX_ADJUSTMENT_RANGE_COUNT = 30;
typedef std::array<rc_adjustment_range_t, RC_MAX_ADJUSTMENT_RANGE_COUNT> rc_adjustment_ranges_t;


class RcAdjustments {
public:
    virtual ~RcAdjustments() = default;
private:
    // RcAdjustments is not copyable or moveable
    RcAdjustments(const RcAdjustments&) = delete;
    RcAdjustments& operator=(const RcAdjustments&) = delete;
    RcAdjustments(RcAdjustments&&) = delete;
    RcAdjustments& operator=(RcAdjustments&&) = delete;
public:
    enum { ADJUSTMENT_RANGE_COUNT_INVALID = -1 };

    enum adjustment_mode_e { ADJUSTMENT_MODE_STEP, ADJUSTMENT_MODE_SELECT };
    struct timed_adjustment_state_t {
        uint32_t timeout_at_milliseconds;
        uint8_t adjustment_range_index;
        bool ready;
    };
    struct continuos_adjustment_state_t {
        uint8_t adjustment_range_index;
        int16_t last_rc_data;
    };
    union adjustment_data_u {
        uint8_t step;
        uint8_t switch_positions;
    };
    struct adjustment_config_t {
        adjustment_e adjustment;
        adjustment_mode_e mode;
        adjustment_data_u data;
    };
    typedef std::array<adjustment_config_t, ADJUSTMENT_FUNCTION_COUNT> adjustment_configs_t;
public:
    explicit RcAdjustments(const adjustment_configs_t* defaultAdjustment_configs);

    void set_adjustment_ranges(const rc_adjustment_ranges_t& adjustment_ranges);
    const rc_adjustment_ranges_t& get_adjustment_ranges() const;

    const rc_adjustment_range_t& get_adjustment_range(size_t index) const;
    void set_adjustment_range(size_t index, const rc_adjustment_range_t& adjustment_range);
    void active_adjustment_range_reset();

    void set_adjustment_configs(const adjustment_configs_t& adjustment_configs);
    const adjustment_configs_t& get_adjustment_configs() const;

    void process_adjustments(const ReceiverBase& receiver, FlightController& flight_controller, Blackbox* blackbox, Cockpit& cockpit, OSD* osd, bool isReceiverSignal);
// public for test code
    int32_t apply_step_adjustment(FlightController& flight_controller, Blackbox* blackbox, rates_t& rates, adjustment_e adjustment, int32_t delta);
    int32_t apply_absolute_adjustment(FlightController& flight_controller, Blackbox* blackbox, rates_t& rates, adjustment_e adjustment, int32_t value);
    uint8_t apply_select_adjustment(FlightController& flight_controller, Cockpit& cockpit, Blackbox* blackbox, OSD* osd, adjustment_e adjustment, uint8_t position);
private:
    enum { ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET = 1};
    void blackbox_log_inflight_adjustment_event(Blackbox* blackbox, adjustment_e adjustment, int32_t newValue);
    void process_stepwise_adjustments(const ReceiverBase& receiver, FlightController& flight_controller, Blackbox* blackbox, rates_t& rates, bool canUseRxData);
    void process_continuous_adjustments(const ReceiverBase& receiver, FlightController& flight_controller, Cockpit& cockpit, Blackbox* blackbox, OSD* osd);
    void calculate_active_adjustment_ranges();
    void beeper_confirmation_beeps(uint8_t beepCount);

    const char *get_range_name();
    int get_range_value();
private:
    int32_t _stepwise_adjustment_count {ADJUSTMENT_RANGE_COUNT_INVALID};
    int32_t _continuos_adjustment_count {};
    std::array<timed_adjustment_state_t, RC_MAX_ADJUSTMENT_RANGE_COUNT> _stepwise_adjustments {};
    std::array<continuos_adjustment_state_t, RC_MAX_ADJUSTMENT_RANGE_COUNT> _continuos_adjustments {};
    std::array<rc_adjustment_range_t, RC_MAX_ADJUSTMENT_RANGE_COUNT> _adjustment_ranges {};
    std::array<adjustment_config_t, ADJUSTMENT_FUNCTION_COUNT> _adjustment_configs {};
    const adjustment_configs_t* _default_adjustment_configs;
};
