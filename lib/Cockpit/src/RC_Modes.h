#pragma once

#include <MspBox.h>
#include <ReceiverBase.h>

#include <algorithm>


struct rc_modes_activation_condition_t {
    receiver_channel_range_t range;
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

    static uint16_t mode_step_to_channel_value(uint8_t step) { return (ReceiverBase::CHANNEL_RANGE_MIN + static_cast<uint16_t>(25 * step)); }
    static uint8_t channel_value_to_step(uint16_t channel_value) { return static_cast<uint8_t>((std::clamp(channel_value, ReceiverBase::CHANNEL_RANGE_MIN, ReceiverBase::CHANNEL_RANGE_MAX) - ReceiverBase::CHANNEL_RANGE_MIN) / ReceiverBase::CHANNEL_RANGE_STEP); }

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
    static bool is_range_usable(const receiver_channel_range_t& range) { return range.start_step < range.end_step; }
    void update_masks_for_mac(const rc_modes_activation_condition_t& mac, MspBox::bitset_t& and_bitset, MspBox::bitset_t& new_bitsets, bool range_active);
    void update_masks_for_sticky_modes(const rc_modes_activation_condition_t& mac, MspBox::bitset_t& and_bitset, MspBox::bitset_t& new_bitset, bool range_active);
private:
    size_t _active_mac_count = 0;
    size_t __active_linked_mac_count = 0;
    MspBox::bitset_t _rc_mode_activation_bitset {};
    MspBox::bitset_t _sticky_modes_ever_disabled_bitset {};
    std::array<uint8_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> _active_mac_array {};
    std::array<uint8_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> _active_linked_mac_array {};
    std::array<rc_modes_activation_condition_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> _mode_activation_conditions {};
};
