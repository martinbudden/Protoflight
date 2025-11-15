#include "Main.h"

#include <Features.h>
#include <MSP_ProtoFlight.h>
#include <MSP_Serial.h>


/*!
Statically allocate the MSP and associated objects.
*/
MSP_SerialBase& Main::createMSP(AHRS& ahrs, FlightController& flightController, Cockpit& cockpit, const ReceiverBase& receiver, const Autopilot& autopilot, const IMU_Filters& imuFilters, Debug& debug, NonVolatileStorage& nvs)
{
    static MSP_ProtoFlight mspProtoFlight(ahrs, flightController, cockpit, receiver, autopilot, imuFilters, debug, nvs);
    static MSP_Stream mspStream(mspProtoFlight);
    static MSP_Serial mspSerial(mspStream);

    return mspSerial;
}
