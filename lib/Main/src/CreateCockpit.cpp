#include "Main.h"

#include <Autopilot.h>
#if defined(USE_BAROMETER_BMP280)
#include <BarometerBMP280.h>
#endif
#include <Cockpit.h>
#include <NonVolatileStorage.h>


Cockpit& Main::createCockpit(ReceiverBase& receiver, FlightController& flightController, Debug& debug, const AHRS_MessageQueue& ahrsMessageQueue, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_BAROMETER_BMP280)
    static BarometerBMP280 barometer(BUS_I2C::BAROMETER_I2C_PINS);
    barometer.init();
    static Autopilot autopilot(ahrsMessageQueue, barometer);
#else
    static Autopilot autopilot(ahrsMessageQueue);
#endif
#if defined USE_ALTITUDE_HOLD
    autopilot.setAutopilotConfig(nvs.loadAutopilotConfig());
    autopilot.setPositionConfig(nvs.loadAutopilotPositionConfig());
    autopilot.setAltitudeHoldConfig(nvs.loadAltitudeHoldConfig());
#endif

    static Cockpit cockpit(receiver, flightController, autopilot, debug, nvs.loadRates(nvs.getCurrentRateProfileIndex()));

    return cockpit;
}
