#include "Main.h"

#include <Features.h>
#include <MSP_ProtoFlight.h>
#include <MSP_Serial.h>


MSP_SerialBase& Main::createMSP(AHRS& ahrs, FlightController& flightController, RadioController& radioController, Debug& debug, NonVolatileStorage& nvs)
{
    static Features features;

    static MSP_ProtoFlight mspProtoFlight(ahrs, flightController, radioController, debug, nvs, features); // NOLINT
    static MSP_Stream mspStream(mspProtoFlight);
    static MSP_Serial mspSerial(mspStream); // NOLINT(misc-const-correctness)

    return mspSerial;
}
