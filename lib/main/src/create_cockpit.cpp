#include "altitude_message_queue.h"
#include "autopilot.h"
#include "cockpit.h"
#if defined(USE_RC_ADJUSTMENTS)
#include "defaults.h"
#endif
#include "main.h"
#include "non_volatile_storage.h"


Cockpit& Main::createCockpit(const AhrsMessageQueue& ahrs_message_queue, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_ALTITUDE_HOLD)
    static AltitudeMessageQueue altitudeMessageQueue;
    static Autopilot autopilot(ahrs_message_queue, altitudeMessageQueue);
    autopilot.set_autopilot_config(nvs.load_autopilot_config());
    autopilot.setPositionHoldConfig(nvs.load_autopilot_position_hold_config());
    autopilot.set_altitude_hold_config(nvs.load_altitude_hold_config());
#else
    static Autopilot autopilot(ahrs_message_queue);
#endif

#if defined(USE_RC_ADJUSTMENTS)
    static Cockpit cockpit(autopilot, &DEFAULTS::RC_ADJUSTMENT_CONFIGS);
    cockpit.get_rc_adjustments().set_adjustment_ranges(nvs.load_rc_adjustment_ranges());
#else
    static Cockpit cockpit(autopilot, nullptr);
#endif
    cockpit.set_rates(nvs.load_rates(nvs.get_current_rate_profile_index()));
    cockpit.set_features(nvs.load_features_config().enabled_features);

    return cockpit;
}
