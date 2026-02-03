#include "AltitudeMessageQueue.h"
#include "Autopilot.h"
#include "Cockpit.h"
#if defined(USE_RC_ADJUSTMENTS)
#include "Defaults.h"
#endif
#include "Main.h"
#include "NonVolatileStorage.h"

#include <ReceiverBase.h>


Cockpit& Main::createCockpit(FlightController& flightController, Debug& debug, IMU_Filters& imuFilters, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
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
    static Cockpit cockpit(flightController, autopilot, imuFilters, debug, nvs, &DEFAULTS::RC_AdjustmentConfigs);
    cockpit.getRC_Adjustments().setAdjustmentRanges(nvs.loadRC_AdjustmentRanges());
#else
    static Cockpit cockpit(flightController, autopilot, imuFilters, debug, nvs, nullptr);
#endif
    cockpit.setCurrentPidProfileIndex(nvs.getCurrentPidProfileIndex());
    cockpit.setCurrentRateProfileIndex(nvs.getCurrentRateProfileIndex());
    cockpit.setRates(nvs.loadRates(nvs.getCurrentRateProfileIndex()), flightController);
    cockpit.setFeatures(nvs.loadFeaturesConfig().enabledFeatures);
    RC_Modes& rcModes = cockpit.getRC_Modes();
    rcModes.setModeActivationConditions(nvs.loadRC_ModeActivationConditions());

#if defined(LIBRARY_RECEIVER_USE_ESPNOW) && false
    const RC_Modes::mode_activation_condition_t macAngle = {
        .modeId = MSP_Box::BOX_ANGLE,
        .auxChannelIndex = ReceiverBase::AUX2,
        .range {
            .startStep = RC_Modes::RANGE_STEP_MID,
            .endStep = RC_Modes::RANGE_STEP_MAX
        },
        .modeLogic {},
        .linkedTo {}
    };
    rcModes.setModeActivationCondition(0, macAngle);

    const RC_Modes::mode_activation_condition_t macAltitudeHold = {
        .modeId = MSP_Box::BOX_ALTHOLD,
        .auxChannelIndex = ReceiverBase::AUX3,
        .range {
            .startStep = RC_Modes::RANGE_STEP_MID,
            .endStep = RC_Modes::RANGE_STEP_MAX
        },
        .modeLogic {},
        .linkedTo {}
    };
    rcModes.setModeActivationCondition(1, macAltitudeHold);
#endif // LIBRARY_RECEIVER_USE_ESPNOW

    return cockpit;
}
