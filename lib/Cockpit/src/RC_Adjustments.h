#pragma once

#include <array>
#include <cstdint>


class RC_Adjustments {
public:
    virtual ~RC_Adjustments() = default;
    RC_Adjustments() = default;
private:
    // RC_Adjustments is not copyable or moveable
    RC_Adjustments(const RC_Adjustments&) = delete;
    RC_Adjustments& operator=(const RC_Adjustments&) = delete;
    RC_Adjustments(RC_Adjustments&&) = delete;
    RC_Adjustments& operator=(RC_Adjustments&&) = delete;
public:
    enum { MAX_ADJUSTMENT_RANGE_COUNT = 30 };
    enum adjustment_mode_e { ADJUSTMENT_MODE_STEP, ADJUSTMENT_MODE_SELECT };
    struct channel_range_t {
        uint8_t startStep;
        uint8_t endStep;
    };
    struct adjustment_range_t {
        // when aux channel is in range...
        uint8_t auxChannelIndex;
        channel_range_t range;
        // ..then apply the adjustment function to the auxSwitchChannel ...
        uint8_t adjustmentConfig;
        uint8_t auxSwitchChannelIndex;
        uint16_t adjustmentCenter;
        uint16_t adjustmentScale;
    };
    struct timed_adjustment_state_t {
        uint32_t timeoutAt;
        uint8_t adjustmentRangeIndex;
        bool ready;
    };
    struct continuos_adjustment_state_t {
        uint8_t adjustmentRangeIndex;
        int16_t lastRcData;
    };
    typedef std::array<adjustment_range_t, MAX_ADJUSTMENT_RANGE_COUNT> adjustment_ranges_t;
public:
    void setAdjustmentRanges(const adjustment_ranges_t& adjustmentRanges);
    const adjustment_ranges_t& getAdjustmentRanges() const;

    const char *getRangeName();
    int getRangeValue();
    void activeRangeReset();
private:
    std::array<adjustment_range_t, MAX_ADJUSTMENT_RANGE_COUNT> _adjustmentRanges {};
};
