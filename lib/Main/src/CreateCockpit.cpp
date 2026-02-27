#include "AltitudeMessageQueue.h"
#include "Autopilot.h"
#include "Cockpit.h"
#if defined(USE_RC_ADJUSTMENTS)
#include "Defaults.h"
#endif
#include "Main.h"
#include "NonVolatileStorage.h"


Cockpit& Main::createCockpit(const AhrsMessageQueue& ahrsMessageQueue, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_ALTITUDE_HOLD)
    static AltitudeMessageQueue altitudeMessageQueue;
    static Autopilot autopilot(ahrsMessageQueue, altitudeMessageQueue);
    autopilot.set_autopilot_config(nvs.load_autopilot_config());
    autopilot.setPositionHoldConfig(nvs.load_autopilot_position_hold_config());
    autopilot.set_altitude_hold_config(nvs.load_altitude_hold_config());
#else
    static Autopilot autopilot(ahrsMessageQueue);
#endif

#if defined(USE_RC_ADJUSTMENTS)
    static Cockpit cockpit(autopilot, &DEFAULTS::RC_ADJUSTMENT_CONFIGS);
    cockpit.getRC_Adjustments().setAdjustmentRanges(nvs.load_rc_adjustment_ranges());
#else
    static Cockpit cockpit(autopilot, nullptr);
#endif
    cockpit.setRates(nvs.load_rates(nvs.get_current_rate_profile_index()));
    cockpit.setFeatures(nvs.load_features_config().enabledFeatures);

    return cockpit;
}
