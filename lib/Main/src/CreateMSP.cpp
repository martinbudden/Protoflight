#include "Main.h"

#include <Features.h>
#include <MSP_ProtoFlight.h>
#include <MSP_Serial.h>


/*!
Statically allocate the MSP and associated objects.
*/
MSP_SerialBase& Main::createMSP(AHRS& ahrs, FlightController& flightController, RadioController& radioController, const ReceiverBase& receiver, const Autopilot& autopilot, Debug& debug, NonVolatileStorage& nvs)
{
    static Features features;

    static MSP_ProtoFlight mspProtoFlight(ahrs, flightController, radioController, receiver, autopilot, debug, nvs, features); // NOLINT
    static MSP_Stream mspStream(mspProtoFlight);
    static MSP_Serial mspSerial(mspStream);

    return mspSerial;
}
