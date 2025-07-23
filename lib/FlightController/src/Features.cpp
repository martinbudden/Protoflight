#include "Features.h"

bool Features::featureIsEnabled(const uint32_t mask) const
{
    switch (mask) {
    case FEATURE_MOTOR_STOP:
        [[fallthrough]];
    case FEATURE_AIRMODE:
        return true;
    default:
        return false;
    }
}

uint32_t Features::enabledFeatures() const
{
    return FEATURE_RX_PPM | FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP| FEATURE_AIRMODE; // NOLINT(hicpp-signed-bitwise)
}

void Features::setFeatures(uint32_t features)
{
    (void)features;
}
