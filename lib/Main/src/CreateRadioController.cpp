#include "Main.h"

#include <Autopilot.h>
#include <RadioController.h>
#include <NonVolatileStorage.h>


RadioController& Main::createRadioController(ReceiverBase& receiver, FlightController& flightController, const BlackboxMessageQueue& blackboxMessageQueue, const AHRS& ahrs, NonVolatileStorage& nvs)
{
    static Autopilot autopilot(blackboxMessageQueue, ahrs);
#if defined USE_ALTITUDE_HOLD
    autopilot.setAltitudeHoldConfig(nvs.loadAltitudeHoldConfig());
#endif

    static RadioController radioController(receiver, flightController, autopilot, nvs.loadRadioControllerRates(nvs.getCurrentRateProfileIndex()));

    return radioController;
}
