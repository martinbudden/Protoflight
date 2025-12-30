#include "RC_Adjustments.h"


void RC_Adjustments::setAdjustmentRanges(const adjustment_ranges_t& adjustmentRanges)
{
    _adjustmentRanges = adjustmentRanges;
}

const RC_Adjustments::adjustment_ranges_t& RC_Adjustments::getAdjustmentRanges() const
{
    return _adjustmentRanges;
}
