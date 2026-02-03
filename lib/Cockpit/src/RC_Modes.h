#pragma once

#include <MSP_Box.h>
#include <ReceiverBase.h>

#include <algorithm>


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
    enum { MAX_MODE_ACTIVATION_CONDITION_COUNT  = 20 };

    static uint16_t modeStepToChannelValue(uint8_t step) { return (ReceiverBase::CHANNEL_RANGE_MIN + static_cast<uint16_t>(25 * step)); }
    static uint8_t channelValueToStep(uint16_t channelValue) { return static_cast<uint8_t>((std::clamp(channelValue, ReceiverBase::CHANNEL_RANGE_MIN, ReceiverBase::CHANNEL_RANGE_MAX) - ReceiverBase::CHANNEL_RANGE_MIN) / ReceiverBase::CHANNEL_RANGE_STEP); }

    static bool pwmIsHigh(uint16_t x) { return x > 1750; }
    static bool pwmIsLow(uint16_t x) { return x < 1250; }
    static bool pwmIsMid(uint16_t x) { return (x > 1250) && (x <1750); }

    enum mode_logic_e { MODE_LOGIC_OR = 0, MODE_LOGIC_AND };
    struct mode_activation_condition_t {
        MSP_Box::id_e modeId;
        uint8_t auxiliaryChannelIndex;
        ReceiverBase::channel_range_t range;
        mode_logic_e modeLogic;
        MSP_Box::id_e linkedTo;
    };
    typedef std::array<mode_activation_condition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT> mode_activation_conditions_t;
public:
    void setModeActivationConditions(const mode_activation_conditions_t& modeActivationConditions);
    const mode_activation_conditions_t& getModeActivationConditions() const { return _modeActivationConditions; }

    void updateActivatedModes(const ReceiverBase& receiver);
    bool isModeActive(MSP_Box::id_e rcMode) const;
    const mode_activation_condition_t& getModeActivationCondition(size_t index) const;
    void setModeActivationCondition(size_t index, const mode_activation_condition_t& modeActivationCondition);
    void analyzeModeActivationConditions();
    bool isModeActivationConditionPresent(MSP_Box::id_e modeId) const;
private:
    bool isModeActivationConditionLinked(MSP_Box::id_e modeId) const;
    //void removeModeActivationCondition(MSP_Box::id_e modeId);
    bool isModeActivationConditionConfigured(const mode_activation_condition_t& mac, const mode_activation_condition_t& emptyMac) const;
    static bool isRangeUsable(const ReceiverBase::channel_range_t& range) { return range.startStep < range.endStep; }
    void updateMasksForMac(const mode_activation_condition_t& mac, MSP_Box::bitset_t& andBitset, MSP_Box::bitset_t& newBitsets, bool rangeActive);
    void updateMasksForStickyModes(const mode_activation_condition_t& mac, MSP_Box::bitset_t& andBitset, MSP_Box::bitset_t& newBitset, bool rangeActive);
private:
    size_t _activeMacCount = 0;
    size_t _activeLinkedMacCount = 0;
    MSP_Box::bitset_t _rcModeActivationBitset {};
    MSP_Box::bitset_t _stickyModesEverDisabledBitset {};
    std::array<uint8_t, MAX_MODE_ACTIVATION_CONDITION_COUNT> _activeMacArray {};
    std::array<uint8_t, MAX_MODE_ACTIVATION_CONDITION_COUNT> _activeLinkedMacArray {};
    std::array<mode_activation_condition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT> _modeActivationConditions {};
};
