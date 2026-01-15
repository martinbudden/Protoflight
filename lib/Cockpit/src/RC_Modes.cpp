#include "RC_Modes.h"
#include "RX.h"

#include <TimeMicroseconds.h>

#include <algorithm>
#include <cassert>
#include <cstring>


void RC_Modes::setModeActivationConditions(const mode_activation_conditions_t& modeActivationConditions)
{
    _modeActivationConditions = modeActivationConditions;
}

const RC_Modes::mode_activation_condition_t& RC_Modes::getModeActivationCondition(size_t index) const
{
    assert(index < MAX_MODE_ACTIVATION_CONDITION_COUNT);
    return _modeActivationConditions[index];
}

void RC_Modes::setModeActivationCondition(size_t index, const mode_activation_condition_t& modeActivationCondition)
{
    if (index < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
        _modeActivationConditions[index] = modeActivationCondition;
    }
}

bool RC_Modes::isModeActive(MSP_Box::id_e rcMode) const
{
    return _rcModeActivationBitset.test(rcMode);
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


bool RC_Modes::isModeActivationConditionPresent(MSP_Box::id_e modeId) const
{
#if false
    for (const auto& mac : _modeActivationConditions) {
        if (mac.modeId == modeId && (isRangeUsable(mac.range) || mac.linkedTo)) {
            return true;
        }
    }
    return false;
#else
    return std::any_of(_modeActivationConditions.begin(), _modeActivationConditions.end(),
        [modeId](const auto& mac) { return mac.modeId == modeId && (isRangeUsable(mac.range) || mac.linkedTo); }
    );
#endif
}

bool RC_Modes::isModeActivationConditionLinked(MSP_Box::id_e modeId) const
{
#if false
    for (const auto& modeActivationCondition : _modeActivationConditions) {
        if (modeActivationCondition.modeId == modeId && modeActivationCondition.linkedTo != 0) {
            return true;
        }
    }
    return false;
#else
    return std::any_of(_modeActivationConditions.begin(), _modeActivationConditions.end(),
        [modeId](const auto& mac) { return (mac.modeId == modeId && mac.linkedTo != 0); }
    );
#endif
}

#if false
void RC_Modes::removeModeActivationCondition(const MSP_Box::id_e modeId)
{
    (void)modeId;

    size_t size = 1;
    size_t index = 0;
    // Shift elements left from index
    std::move(_modeActivationConditions.begin() + index + 1, _modeActivationConditions.begin() + size, _modeActivationConditions.begin() + index);

    //const auto newEnd = std::remove(_modeActivationConditions.begin(), _modeActivationConditions.end(), modeId);
    //const size_t size = std::distance(_modeActivationConditions.begin(), newEnd);
    // reset unused slots
    //std::fill(newEnd, _modeActivationConditions.end(), 0);
}
#endif

bool RC_Modes::isModeActivationConditionConfigured(const mode_activation_condition_t& mac, const mode_activation_condition_t& emptyMac) const
{
    if (memcmp(&mac, &emptyMac, sizeof(emptyMac))) {
        return true;
    }
    return false;
}

/*!
Build the list of used modeActivationConditions indices
We can then use this to speed up processing by only evaluating used conditions
*/
void RC_Modes::analyzeModeActivationConditions()
{
    mode_activation_condition_t emptyMac {};

    _activeMacCount = 0;
    _activeLinkedMacCount = 0;

    uint8_t ii = 0;
    for (const auto& modeActivationCondition : _modeActivationConditions) {
        if (modeActivationCondition.linkedTo) {
            _activeLinkedMacArray[_activeLinkedMacCount++] = ii;
        } else if (isModeActivationConditionConfigured(modeActivationCondition, emptyMac)) {
            _activeMacArray[_activeMacCount++] = ii;
        }
        ++ii;
    }
}

/*
 *  updateMasksForMac:
 *
 *  The following are the possible logic states at each MAC update:
 *      AND     NEW
 *      ---     ---
 *       F       F      - no previous AND macs evaluated, no previous active OR macs
 *       F       T      - at least 1 previous active OR mac (***this state is latched True***)
 *       T       F      - all previous AND macs active, no previous active OR macs
 *       T       T      - at least 1 previous inactive AND mac, no previous active OR macs
 */
void RC_Modes::updateMasksForMac(const mode_activation_condition_t& mac, MSP_Box::bitset_t& andBitset, MSP_Box::bitset_t& newBitset, bool rangeActive)
{
    if (andBitset.test(mac.modeId) || !newBitset.test(mac.modeId)) {
        const bool bAnd = mac.modeLogic == MODE_LOGIC_AND;
        if (!bAnd) { // OR mode_activation_condition
            if (rangeActive) {
                andBitset.reset(mac.modeId);
                newBitset.set(mac.modeId);
            }
        } else { // AND mode_activation_condition
            andBitset.set(mac.modeId);
            if (!rangeActive) {
                newBitset.set(mac.modeId);
            }
        }
    }
}

void RC_Modes::updateMasksForStickyModes(const mode_activation_condition_t& mac, MSP_Box::bitset_t& andBitset, MSP_Box::bitset_t& newBitset, bool rangeActive)
{
    enum { STICKY_MODE_BOOT_DELAY_US = 5000000 }; // 5 seconds
    if (isModeActive(mac.modeId)) {
        andBitset.reset(mac.modeId);
        newBitset.set(mac.modeId);
    } else {
        if (_stickyModesEverDisabledBitset.test(mac.modeId)) {
            updateMasksForMac(mac, andBitset, newBitset, rangeActive);
        } else {
            if (timeUs() >= STICKY_MODE_BOOT_DELAY_US && !rangeActive) { // cppcheck-suppress knownConditionTrueFalse
                _stickyModesEverDisabledBitset.set(mac.modeId);
            }
        }
    }
}

void RC_Modes::updateActivatedModes(const ReceiverBase& receiver)
{
    MSP_Box::bitset_t newBitset {};
    MSP_Box::bitset_t andBitset {};
    MSP_Box::bitset_t stickyModes {};
    stickyModes.set(MSP_Box::BOX_PARALYZE);

    // determine which conditions set/clear the mode
    size_t ii = 0;
    for (const auto& modeActivationCondition : _modeActivationConditions) {
        if (stickyModes.test(modeActivationCondition.modeId)) {
            const bool rangeActive = receiver.isRangeActive(modeActivationCondition.auxiliaryChannelIndex, modeActivationCondition.range);
            updateMasksForStickyModes(modeActivationCondition, andBitset, newBitset, rangeActive);
        } else if (modeActivationCondition.modeId < MSP_Box::BOX_COUNT) {
            const bool rangeActive = receiver.isRangeActive(modeActivationCondition.auxiliaryChannelIndex, modeActivationCondition.range);
            updateMasksForMac(modeActivationCondition, andBitset, newBitset, rangeActive);
        }
        ++ii;
        if (ii == _activeMacCount) {
            break;
        }
    }

    // Update linked modes
    ii = 0;
    for (const auto& modeActivationCondition : _modeActivationConditions) {
        const bool rangeActive = andBitset.test(modeActivationCondition.linkedTo) != newBitset.test(modeActivationCondition.linkedTo);
        updateMasksForMac(modeActivationCondition, andBitset, newBitset, rangeActive);
        ++ii;
        if (ii == _activeLinkedMacCount) {
            break;
        }
    }

    _rcModeActivationBitset = newBitset ^ andBitset;
#if false
    enum { ANGLE_MODE_CHANNEL = ReceiverBase::AUX2, ALTITUDE_HOLD_MODE_CHANNEL = ReceiverBase::AUX3 };
    if (receiver.getChannelPWM(ANGLE_MODE_CHANNEL)) {
        _rcModeActivationBitset.set(MSP_Box::BOX_ANGLE);
    }
    if (receiver.getChannelPWM(ALTITUDE_HOLD_MODE_CHANNEL)) {
        _rcModeActivationBitset.set(MSP_Box::BOX_ALTHOLD);
    }
#endif

    //airmodeEnabled = featureIsEnabled(FEATURE_AIRMODE) || IS_RC_MODE_ACTIVE(BOX_AIRMODE);
}
