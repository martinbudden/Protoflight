#pragma once

#include <MSP_Box.h>
#include <ReceiverBase.h>

#include <algorithm>


struct rc_modes_activation_condition_t {
    receiver_channel_range_t range;
    uint8_t modeId;
    uint8_t auxiliaryChannelIndex;
    uint8_t modeLogic;
    uint8_t linkedTo;
};


static constexpr uint8_t RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT  = 20;
typedef std::array<rc_modes_activation_condition_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> rc_modes_activation_condition_array_t;


class RC_Modes {
public:
    virtual ~RC_Modes() = default;
    RC_Modes() = default;
private:
    // RC_Modes is not copyable or moveable
    RC_Modes(const RC_Modes&) = delete;
    RC_Modes& operator=(const RC_Modes&) = delete;
    RC_Modes(RC_Modes&&) = delete;
    RC_Modes& operator=(RC_Modes&&) = delete;
public:

    static uint16_t modeStepToChannelValue(uint8_t step) { return (ReceiverBase::CHANNEL_RANGE_MIN + static_cast<uint16_t>(25 * step)); }
    static uint8_t channelValueToStep(uint16_t channelValue) { return static_cast<uint8_t>((std::clamp(channelValue, ReceiverBase::CHANNEL_RANGE_MIN, ReceiverBase::CHANNEL_RANGE_MAX) - ReceiverBase::CHANNEL_RANGE_MIN) / ReceiverBase::CHANNEL_RANGE_STEP); }

    static bool pwmIsHigh(uint16_t x) { return x > 1750; }
    static bool pwmIsLow(uint16_t x) { return x < 1250; }
    static bool pwmIsMid(uint16_t x) { return (x > 1250) && (x <1750); }

    enum mode_logic_e { MODE_LOGIC_OR = 0, MODE_LOGIC_AND };
public:
    void setModeActivationConditions(const rc_modes_activation_condition_array_t& modeActivationConditions);
    const rc_modes_activation_condition_array_t& getModeActivationConditions() const { return _modeActivationConditions; }

    void updateActivatedModes(const ReceiverBase& receiver);
    bool isModeActive(uint8_t rcMode) const;
    const rc_modes_activation_condition_t& getModeActivationCondition(size_t index) const;
    void setModeActivationCondition(size_t index, const rc_modes_activation_condition_t& modeActivationCondition);
    void analyzeModeActivationConditions();
    bool isModeActivationConditionPresent(uint8_t modeId) const;
private:
    bool isModeActivationConditionLinked(uint8_t modeId) const;
    //void removeModeActivationCondition(uint8_t modeId);
    bool isModeActivationConditionConfigured(const rc_modes_activation_condition_t& mac, const rc_modes_activation_condition_t& emptyMac) const;
    static bool isRangeUsable(const receiver_channel_range_t& range) { return range.start_step < range.end_step; }
    void updateMasksForMac(const rc_modes_activation_condition_t& mac, MSP_Box::bitset_t& andBitset, MSP_Box::bitset_t& newBitsets, bool rangeActive);
    void updateMasksForStickyModes(const rc_modes_activation_condition_t& mac, MSP_Box::bitset_t& andBitset, MSP_Box::bitset_t& newBitset, bool rangeActive);
private:
    size_t _activeMacCount = 0;
    size_t _activeLinkedMacCount = 0;
    MSP_Box::bitset_t _rcModeActivationBitset {};
    MSP_Box::bitset_t _stickyModesEverDisabledBitset {};
    std::array<uint8_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> _activeMacArray {};
    std::array<uint8_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> _activeLinkedMacArray {};
    std::array<rc_modes_activation_condition_t, RC_MODES_MAX_MODE_ACTIVATION_CONDITION_COUNT> _modeActivationConditions {};
};
