#include "Autopilot.h"
#include "Cockpit.h"
#include "Main.h"
#include "NonVolatileStorage.h"

#if defined(USE_BAROMETER_BMP280)
#include <BarometerBMP280.h>
#endif


Cockpit& Main::createCockpit(ReceiverBase& receiver, FlightController& flightController, Debug& debug, IMU_Filters& imuFilters, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_BAROMETER_BMP280)
    static BarometerBMP280 barometer(BUS_I2C::BAROMETER_I2C_PINS);
    barometer.init();
    static Autopilot autopilot(flightController.getAHRS_MessageQueue(), barometer);
#else
    static Autopilot autopilot(flightController.getAHRS_MessageQueue());
#endif
#if defined USE_ALTITUDE_HOLD
    autopilot.setAutopilotConfig(nvs.loadAutopilotConfig());
    autopilot.setPositionConfig(nvs.loadAutopilotPositionConfig());
    autopilot.setAltitudeHoldConfig(nvs.loadAltitudeHoldConfig());
#endif

    static Cockpit cockpit(receiver, flightController, autopilot, imuFilters, debug, nvs);
    cockpit.setCurrentPidProfileIndex(nvs.getCurrentPidProfileIndex());
    cockpit.setCurrentRateProfileIndex(nvs.getCurrentRateProfileIndex());
    cockpit.setRates(nvs.loadRates(nvs.getCurrentRateProfileIndex()));

    return cockpit;
}
