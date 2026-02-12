#include "AltitudeMessageQueue.h"
#include "Autopilot.h"
#include "Cockpit.h"
#if defined(USE_RC_ADJUSTMENTS)
#include "Defaults.h"
#endif
#include "Main.h"
#include "NonVolatileStorage.h"

#include <ReceiverBase.h>


Cockpit& Main::createCockpit(RcModes& rc_modes, FlightController& flightController, Debug& debug, IMU_Filters& imuFilters, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_ALTITUDE_HOLD)
    static AltitudeMessageQueue altitudeMessageQueue;
    static Autopilot autopilot(flightController.getAHRS_MessageQueue(), altitudeMessageQueue);
    autopilot.setAutopilotConfig(nvs.loadAutopilotConfig());
    autopilot.setPositionConfig(nvs.loadAutopilotPositionConfig());
    autopilot.setAltitudeHoldConfig(nvs.loadAltitudeHoldConfig());
#else
    static Autopilot autopilot(flightController.getAHRS_MessageQueue());
#endif

#if defined(USE_RC_ADJUSTMENTS)
    static Cockpit cockpit(rc_modes, flightController, autopilot, imuFilters, debug, &DEFAULTS::RC_AdjustmentConfigs);
    cockpit.getRC_Adjustments().setAdjustmentRanges(nvs.loadRC_AdjustmentRanges());
#else
    static Cockpit cockpit(rc_modes, flightController, autopilot, imuFilters, debug, nullptr);
#endif
    cockpit.setRates(nvs.loadRates(nvs.getCurrentRateProfileIndex()));
    cockpit.setFeatures(nvs.loadFeaturesConfig().enabledFeatures);
    rc_modes.set_mode_activation_conditions(nvs.load_rc_mode_activation_conditions());

#if defined(LIBRARY_RECEIVER_USE_ESPNOW) && false
    const rc_modes_activation_condition_t macAngle = {
        .mode_id = MspBox::BOX_ANGLE,
        .aux_channel_index = ReceiverBase::AUX2,
        .range {
            .start_step = RcModes::RANGE_STEP_MID,
            .end_step = RcModes::RANGE_STEP_MAX
        },
        .mode_logic {},
        .linked_to {}
    };
    rcModes.set_mode_activation_condition(0, macAngle);

    const rc_modes_activation_condition_t macAltitudeHold = {
        .mode_id = MspBox::BOX_ALTHOLD,
        .aux_channel_index = ReceiverBase::AUX3,
        .range {
            .start_step = RcModes::RANGE_STEP_MID,
            .end_step = RcModes::RANGE_STEP_MAX
        },
        .mode_logic {},
        .linked_to {}
    };
    rcModes.set_mode_activation_condition(1, macAltitudeHold);
#endif // LIBRARY_RECEIVER_USE_ESPNOW

    return cockpit;
}
