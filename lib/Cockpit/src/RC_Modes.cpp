#include "RC_Modes.h"
#include "RX.h"

#include <TimeMicroseconds.h>

#include <algorithm>
#include <cassert>
#include <cstring>


void RcModes::set_mode_activation_conditions(const rc_modes_activation_condition_array_t& mode_activation_conditions)
{
    _mode_activation_conditions = mode_activation_conditions;
}

const rc_modes_activation_condition_t& RcModes::get_mode_activation_condition(size_t index) const
{
    assert(index < RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT);
    return _mode_activation_conditions[index];
}

void RcModes::set_mode_activation_condition(size_t index, const rc_modes_activation_condition_t& mode_activation_condition)
{
    if (index < RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT) {
        _mode_activation_conditions[index] = mode_activation_condition;
    }
}

bool RcModes::is_mode_active(uint8_t rcMode) const
{
    return _rc_mode_activation_bitset.test(rcMode);
/*!!
    if (rcMode == BOX_OSD) {
        return false;
    }
    if (rcMode == BOX_STICK_COMMAND_DISABLE) {
        return false;
    }
    return false; // !!TODO rcMode
*/
}


bool RcModes::is_mode_activation_condition_present(uint8_t mode_id) const
{
#if false
    for (const auto& mac : _mode_activation_conditions) {
        if (mac.mode_id == mode_id && (is_range_usable(mac.range) || mac.linked_to)) {
            return true;
        }
    }
    return false;
#else
    return std::any_of(_mode_activation_conditions.begin(), _mode_activation_conditions.end(),
        [mode_id](const auto& mac) { return mac.mode_id == mode_id && (is_range_usable(mac.range) || mac.linked_to); }
    );
#endif
}

bool RcModes::is_mode_activation_condition_linked(uint8_t mode_id) const
{
#if false
    for (const auto& mode_activation_condition : _mode_activation_conditions) {
        if (mode_activation_condition.mode_id == mode_id && mode_activation_condition.linked_to != 0) {
            return true;
        }
    }
    return false;
#else
    return std::any_of(_mode_activation_conditions.begin(), _mode_activation_conditions.end(),
        [mode_id](const auto& mac) { return (mac.mode_id == mode_id && mac.linked_to != 0); }
    );
#endif
}

#if false
void RcModes::remove_mode_activation_condition(const uint8_t mode_id)
{
    (void)mode_id;

    size_t size = 1;
    size_t index = 0;
    // Shift elements left from index
    std::move(_mode_activation_conditions.begin() + index + 1, _mode_activation_conditions.begin() + size, _mode_activation_conditions.begin() + index);

    //const auto newEnd = std::remove(_mode_activation_conditions.begin(), _mode_activation_conditions.end(), mode_id);
    //const size_t size = std::distance(_mode_activation_conditions.begin(), newEnd);
    // reset unused slots
    //std::fill(newEnd, _mode_activation_conditions.end(), 0);
}
#endif

bool RcModes::is_mode_activation_condition_configured(const rc_modes_activation_condition_t& mac, const rc_modes_activation_condition_t& empty_mac) const
{
    if (memcmp(&mac, &empty_mac, sizeof(empty_mac))) {
        return true;
    }
    return false;
}

/*!
Build the list of used mode_activation_conditions indices
We can then use this to speed up processing by only evaluating used conditions
*/
void RcModes::analyze_mode_activation_conditions()
{
    rc_modes_activation_condition_t empty_mac {};

    _active_mac_count = 0;
    __active_linked_mac_count = 0;

    uint8_t ii = 0;
    for (const auto& mode_activation_condition : _mode_activation_conditions) {
        if (mode_activation_condition.linked_to) {
            _active_linked_mac_array[__active_linked_mac_count] = ii;
            ++__active_linked_mac_count;
        } else if (is_mode_activation_condition_configured(mode_activation_condition, empty_mac)) {
            _active_mac_array[_active_mac_count] = ii;
            ++_active_mac_count;
        }
        ++ii;
    }
}

/*
 *  update_masks_for_mac:
 *
 *  The following are the possible logic states at each MAC update:
 *      AND     NEW
 *      ---     ---
 *       F       F      - no previous AND macs evaluated, no previous active OR macs
 *       F       T      - at least 1 previous active OR mac (***this state is latched True***)
 *       T       F      - all previous AND macs active, no previous active OR macs
 *       T       T      - at least 1 previous inactive AND mac, no previous active OR macs
 */
void RcModes::update_masks_for_mac(const rc_modes_activation_condition_t& mac, MspBox::bitset_t& and_bitset, MspBox::bitset_t& new_bitset, bool range_active)
{
    if (and_bitset.test(mac.mode_id) || !new_bitset.test(mac.mode_id)) {
        const bool bAnd = mac.mode_logic == MODE_LOGIC_AND;
        if (!bAnd) { // OR mode_activation_condition
            if (range_active) {
                and_bitset.reset(mac.mode_id);
                new_bitset.set(mac.mode_id);
            }
        } else { // AND mode_activation_condition
            and_bitset.set(mac.mode_id);
            if (!range_active) {
                new_bitset.set(mac.mode_id);
            }
        }
    }
}

void RcModes::update_masks_for_sticky_modes(const rc_modes_activation_condition_t& mac, MspBox::bitset_t& and_bitset, MspBox::bitset_t& new_bitset, bool range_active)
{
    enum { STICKY_MODE_BOOT_DELAY_US = 5000000 }; // 5 seconds
    if (is_mode_active(mac.mode_id)) {
        and_bitset.reset(mac.mode_id);
        new_bitset.set(mac.mode_id);
    } else {
        if (_sticky_modes_ever_disabled_bitset.test(mac.mode_id)) {
            update_masks_for_mac(mac, and_bitset, new_bitset, range_active);
        } else {
            if (timeUs() >= STICKY_MODE_BOOT_DELAY_US && !range_active) { // cppcheck-suppress knownConditionTrueFalse
                _sticky_modes_ever_disabled_bitset.set(mac.mode_id);
            }
        }
    }
}

void RcModes::update_activated_modes(const ReceiverBase& receiver)
{
    MspBox::bitset_t new_bitset {};
    MspBox::bitset_t and_bitset {};
    MspBox::bitset_t stickyModes {};
    stickyModes.set(MspBox::BOX_PARALYZE);

    // determine which conditions set/clear the mode
    size_t ii = 0;
    for (const auto& mode_activation_condition : _mode_activation_conditions) {
        if (stickyModes.test(mode_activation_condition.mode_id)) {
            const bool range_active = receiver.is_range_active(mode_activation_condition.auxiliary_channel_index, mode_activation_condition.range);
            update_masks_for_sticky_modes(mode_activation_condition, and_bitset, new_bitset, range_active);
        } else if (mode_activation_condition.mode_id < MspBox::BOX_COUNT) {
            const bool range_active = receiver.is_range_active(mode_activation_condition.auxiliary_channel_index, mode_activation_condition.range);
            update_masks_for_mac(mode_activation_condition, and_bitset, new_bitset, range_active);
        }
        ++ii;
        if (ii == _active_mac_count) {
            break;
        }
    }

    // Update linked modes
    ii = 0;
    for (const auto& mode_activation_condition : _mode_activation_conditions) {
        const bool range_active = and_bitset.test(mode_activation_condition.linked_to) != new_bitset.test(mode_activation_condition.linked_to);
        update_masks_for_mac(mode_activation_condition, and_bitset, new_bitset, range_active);
        ++ii;
        if (ii == __active_linked_mac_count) {
            break;
        }
    }

    _rc_mode_activation_bitset = new_bitset ^ and_bitset;
#if false
    enum { ANGLE_MODE_CHANNEL = ReceiverBase::AUX2, ALTITUDE_HOLD_MODE_CHANNEL = ReceiverBase::AUX3 };
    if (receiver.get_channel_pwm(ANGLE_MODE_CHANNEL)) {
        _rc_mode_activation_bitset.set(MspBox::BOX_ANGLE);
    }
    if (receiver.get_channel_pwm(ALTITUDE_HOLD_MODE_CHANNEL)) {
        _rc_mode_activation_bitset.set(MspBox::BOX_ALTHOLD);
    }
#endif

    //airmodeEnabled = featureIsEnabled(FEATURE_AIRMODE) || IS_RC_MODE_ACTIVE(BOX_AIRMODE);
}
