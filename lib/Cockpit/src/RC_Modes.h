#pragma once

#include "MspBox.h"

class ReceiverBase;

struct rc_modes_activation_condition_t {
    uint8_t range_start;
    uint8_t range_end;
    uint8_t mode_id;
    uint8_t auxiliary_channel_index;
    uint8_t mode_logic;
    uint8_t linked_to;
};


static constexpr uint8_t RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT  = 20;
typedef std::array<rc_modes_activation_condition_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> rc_modes_activation_condition_array_t;


class RcModes {
public:
    virtual ~RcModes() = default;
    RcModes() = default;
private:
    // RcModes is not copyable or moveable
    RcModes(const RcModes&) = delete;
    RcModes& operator=(const RcModes&) = delete;
    RcModes(RcModes&&) = delete;
    RcModes& operator=(RcModes&&) = delete;
public:

    /*! 
    Steps are 25 apart
        a value of 0 corresponds to a channel value of 900 or less
        a value of 48 corresponds to a channel value of 2100 or more
    48 steps between 900 and 2100
    */
    static constexpr uint16_t CHANNEL_RANGE_MIN = 900;
    static constexpr uint16_t CHANNEL_RANGE_MID = 1500;
    static constexpr uint16_t CHANNEL_RANGE_MAX = 2100;

    static constexpr uint16_t CHANNEL_RANGE_STEP = 25;
    static constexpr uint16_t RANGE_STEP_MIN = 0;
    static constexpr uint16_t RANGE_STEP_MID = ((CHANNEL_RANGE_MID - CHANNEL_RANGE_MIN) / CHANNEL_RANGE_STEP);
    static constexpr uint16_t RANGE_STEP_MAX = ((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / CHANNEL_RANGE_STEP);

    static uint16_t mode_step_to_channel_value(uint8_t step);
    static uint8_t channel_value_to_step(uint16_t channel_value);

    static bool pwm_is_high(uint16_t x) { return x > 1750; }
    static bool pwm_is_low(uint16_t x) { return x < 1250; }
    static bool pwm_is_mid(uint16_t x) { return (x > 1250) && (x <1750); }

    enum mode_logic_e { MODE_LOGIC_OR = 0, MODE_LOGIC_AND };
public:
    void set_mode_activation_conditions(const rc_modes_activation_condition_array_t& mode_activation_conditions);
    const rc_modes_activation_condition_array_t& get_mode_activation_conditions() const { return _mode_activation_conditions; }

    void update_activated_modes(const ReceiverBase& receiver);
    bool is_mode_active(uint8_t rcMode) const;
    const rc_modes_activation_condition_t& get_mode_activation_condition(size_t index) const;
    void set_mode_activation_condition(size_t index, const rc_modes_activation_condition_t& mode_activation_condition);
    void analyze_mode_activation_conditions();
    bool is_mode_activation_condition_present(uint8_t mode_id) const;
private:
    bool is_mode_activation_condition_linked(uint8_t mode_id) const;
    //void remove_mode_activation_condition(uint8_t mode_id);
    bool is_mode_activation_condition_configured(const rc_modes_activation_condition_t& mac, const rc_modes_activation_condition_t& empty_mac) const;
    static bool is_range_usable(uint8_t range_start, uint8_t range_end);
    static bool is_range_active(uint16_t channel_value, uint8_t range_start, uint8_t range_end);
    void update_masks_for_mac(const rc_modes_activation_condition_t& mac, MspBox::bitset_t& and_bitset, MspBox::bitset_t& new_bitsets, bool range_active);
    void update_masks_for_sticky_modes(const rc_modes_activation_condition_t& mac, MspBox::bitset_t& and_bitset, MspBox::bitset_t& new_bitset, bool range_active);
private:
    size_t _active_mac_count = 0;
    size_t _active_linked_mac_count = 0;
    MspBox::bitset_t _rc_mode_activation_bitset {};
    MspBox::bitset_t _sticky_modes_ever_disabled_bitset {};
    std::array<uint8_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> _active_mac_array {};
    std::array<uint8_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> _active_linked_mac_array {};
    std::array<rc_modes_activation_condition_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> _mode_activation_conditions {};
};
