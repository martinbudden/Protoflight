#include "Main.h"

#include <DisplayPortNull.h>
#include <OSD.h>


/*!
Statically allocate the OSD and associated objects.
*/
OSD& Main::createOSD(const FlightController& flightController, const Cockpit& cockpit, Debug& debug)
{
    static OSD osd(flightController, cockpit, debug);
    static DisplayPortNull displayPort;
    osd.init(&displayPort, DisplayPortBase::DEVICE_TYPE_NONE);

    return osd;
}
