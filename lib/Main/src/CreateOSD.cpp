#include "Main.h"

#include <OSD.h>


/*!
Statically allocate the OSD and associated objects.
*/
OSD& Main::createOSD(const FlightController& flightController, const Cockpit& cockpit, Debug& debug)
{
    static OSD osd(flightController, cockpit, debug);
    //!!TODO:init OSD with displayPort
    //osd.init();

    return osd;
}
