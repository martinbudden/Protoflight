#include "MSP_ProtoFlight.h"
#include "Main.h"

#include <Features.h>
#include <MSP_Serial.h>


/*!
Statically allocate the MSP and associated objects.
*/
MSP_SerialBase* Main::createMSP(AHRS& ahrs, FlightController& flightController, Cockpit& cockpit, const ReceiverBase& receiver, const Autopilot& autopilot, const IMU_Filters& imuFilters, Debug& debug, NonVolatileStorage& nvs, Blackbox* blackbox, VTX_Base*vtx, OSD* osd)
{
#if defined(USE_MSP)
    static MSP_ProtoFlight mspProtoFlight(ahrs, flightController, cockpit, receiver, autopilot, imuFilters, debug, nvs, blackbox, vtx, osd);
    static MSP_Stream mspStream(mspProtoFlight);
    static MSP_Serial mspSerial(mspStream);

    return &mspSerial;
#else
    (void)ahrs;
    (void)flightController;
    (void)cockpit;
    (void)receiver;
    (void)autopilot;
    (void)imuFilters;
    (void)debug;
    (void)nvs;
    (void)blackbox;
    (void)vtx;
    (void)osd;

    return nullptr;
#endif
}
