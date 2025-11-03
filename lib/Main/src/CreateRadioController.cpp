#include "Main.h"

#include <Autopilot.h>
#if defined(USE_BAROMETER_BMP280)
#include <Barometer_BMP280.h>
#endif
#include <NonVolatileStorage.h>
#include <RadioController.h>


RadioController& Main::createRadioController(ReceiverBase& receiver, FlightController& flightController, Debug& debug, const AHRS_MessageQueue& ahrsMessageQueue, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_BAROMETER_BMP280)
    static Barometer_BMP280 barometer(BUS_I2C::BAROMETER_I2C_PINS);
    barometer.init();
    static Autopilot autopilot(ahrsMessageQueue, barometer);
#else
    static Autopilot autopilot(ahrsMessageQueue);
#endif
#if defined USE_ALTITUDE_HOLD
    autopilot.setAltitudeHoldConfig(nvs.loadAltitudeHoldConfig());
#endif

    static RadioController radioController(receiver, flightController, autopilot, debug, nvs.loadRadioControllerRates(nvs.getCurrentRateProfileIndex()));

    return radioController;
}
