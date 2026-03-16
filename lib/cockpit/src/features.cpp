#include "features.h"

bool Features::is_enabled(const uint32_t mask) const
{
    return _featureMask & mask;
}

void Features::set(uint32_t mask)
{
    _featureMask |= mask;
}

void Features::clear(uint32_t mask)
{
    _featureMask &= ~mask;
}

uint32_t Features::enabled_features() const
{
    return FEATURE_RX_PPM | FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP| FEATURE_AIRMODE; // NOLINT(hicpp-signed-bitwise)
}

