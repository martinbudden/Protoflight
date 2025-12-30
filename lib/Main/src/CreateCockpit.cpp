#include "AltitudeMessageQueue.h"
#include "Autopilot.h"
#include "Cockpit.h"
#include "Main.h"
#include "NonVolatileStorage.h"


Cockpit& Main::createCockpit(ReceiverBase& receiver, FlightController& flightController, Debug& debug, IMU_Filters& imuFilters, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
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

    static Cockpit cockpit(receiver, flightController, autopilot, imuFilters, debug, nvs);
    cockpit.setCurrentPidProfileIndex(nvs.getCurrentPidProfileIndex());
    cockpit.setCurrentRateProfileIndex(nvs.getCurrentRateProfileIndex());
    cockpit.setRates(nvs.loadRates(nvs.getCurrentRateProfileIndex()));
    cockpit.getRC_Modes().setModeActivationConditions(nvs.loadRC_ModeActivationConditions());
    cockpit.getRC_Adjustments().setAdjustmentRanges(nvs.loadRC_AdjustmentRanges());
    cockpit.setFeatures(nvs.loadFeaturesConfig().enabledFeatures);

    return cockpit;
}
