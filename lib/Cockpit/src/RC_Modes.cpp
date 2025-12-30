#include "RC_Modes.h"
#include "RX.h"

#include <ReceiverBase.h>
#include <TimeMicroseconds.h>

#include <algorithm>
#include <cassert>
#include <cstring>


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
    return _rcModeActivationMask.test(rcMode);
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


bool RC_Modes::isModeActivationConditionPresent(MSP_Box::id_e modeId)
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

bool RC_Modes::isModeActivationConditionLinked(MSP_Box::id_e modeId)
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

bool RC_Modes::isModeActivationConditionConfigured(const mode_activation_condition_t& mac, const mode_activation_condition_t& emptyMac)
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
void RC_Modes::updateMasksForMac(const mode_activation_condition_t& mac, MSP_Box::bitset_t& andMask, MSP_Box::bitset_t& newMask, bool rangeActive)
{
    if (andMask.test(mac.modeId) || !newMask.test(mac.modeId)) {
        const bool bAnd = mac.modeLogic == MODE_LOGIC_AND;
        if (!bAnd) { // OR mode_activation_condition
            if (rangeActive) {
                andMask.reset(mac.modeId);
                newMask.set(mac.modeId);
            }
        } else { // AND mode_activation_condition
            andMask.set(mac.modeId);
            if (!rangeActive) {
                newMask.set(mac.modeId);
            }
        }
    }
}

void RC_Modes::updateMasksForStickyModes(const mode_activation_condition_t& mac, MSP_Box::bitset_t& andMask, MSP_Box::bitset_t& newMask, bool rangeActive)
{
    enum { STICKY_MODE_BOOT_DELAY_US = 5000000 }; // 5 seconds
    if (isModeActive(mac.modeId)) {
        andMask.reset(mac.modeId);
        newMask.set(mac.modeId);
    } else {
        if (_stickyModesEverDisabled.test(mac.modeId)) {
            updateMasksForMac(mac, andMask, newMask, rangeActive);
        } else {
            if (timeUs() >= STICKY_MODE_BOOT_DELAY_US && !rangeActive) {
                _stickyModesEverDisabled.set(mac.modeId);
            }
        }
    }
}

bool RC_Modes::isRangeActive(const ReceiverBase& receiver, uint8_t auxChannelIndex, const channel_range_t& range)
{
    if (!isRangeUsable(range)) {
        return false;
    }
    const uint16_t channelValue = receiver.getAuxiliaryChannel(auxChannelIndex);
    return (channelValue >= CHANNEL_RANGE_MIN + (range.startStep * 25) && channelValue < CHANNEL_RANGE_MIN + (range.endStep * 25));
}

void RC_Modes::updateActivatedModes(const ReceiverBase& receiver)
{
    MSP_Box::bitset_t newMask {};
    MSP_Box::bitset_t andMask {};
    MSP_Box::bitset_t stickyModes {};
    stickyModes.set(MSP_Box::BOX_PARALYZE);

    // determine which conditions set/clear the mode
    size_t ii = 0;
    for (const auto& modeActivationCondition : _modeActivationConditions) {
        if (stickyModes.test(modeActivationCondition.modeId)) {
            const bool rangeActive = isRangeActive(receiver, modeActivationCondition.auxChannelIndex, modeActivationCondition.range);
            updateMasksForStickyModes(modeActivationCondition, andMask, newMask, rangeActive);
        } else if (modeActivationCondition.modeId < MSP_Box::BOX_COUNT) {
            const bool rangeActive = isRangeActive(receiver, modeActivationCondition.auxChannelIndex, modeActivationCondition.range);
            updateMasksForMac(modeActivationCondition, andMask, newMask, rangeActive);
        }
        ++ii;
        if (ii == _activeMacCount) {
            break;
        }
    }

    // Update linked modes
    ii = 0;
    for (const auto& modeActivationCondition : _modeActivationConditions) {
        const bool rangeActive = andMask.test(modeActivationCondition.linkedTo) != newMask.test(modeActivationCondition.linkedTo);
        updateMasksForMac(modeActivationCondition, andMask, newMask, rangeActive);
        ++ii;
        if (ii == _activeLinkedMacCount) {
            break;
        }
    }

    _rcModeActivationMask = newMask ^ andMask;

    //airmodeEnabled = featureIsEnabled(FEATURE_AIRMODE) || IS_RC_MODE_ACTIVE(BOX_AIRMODE);
}
