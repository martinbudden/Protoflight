#include "Main.h"

#include <Autopilot.h>
#include <NonVolatileStorage.h>
#include <RadioController.h>


RadioController& Main::createRadioController(ReceiverBase& receiver, FlightController& flightController, const AHRS_MessageQueue& ahrsMessageQueue, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
    static Autopilot autopilot(ahrsMessageQueue);
#if defined USE_ALTITUDE_HOLD
    autopilot.setAltitudeHoldConfig(nvs.loadAltitudeHoldConfig());
#endif

    static RadioController radioController(receiver, flightController, autopilot, nvs.loadRadioControllerRates(nvs.getCurrentRateProfileIndex()));

    return radioController;
}
