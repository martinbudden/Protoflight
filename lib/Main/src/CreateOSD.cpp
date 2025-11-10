#include "Main.h"

#include <OSD.h>


/*!
Statically allocate the OSD and associated objects.
*/
OSD& Main::createOSD(const FlightController& flightController, Debug& debug)
{
    (void)debug;
    static OSD osd(flightController);
    //!!TODO:init OSD with displayPort
    //osd.init();

    return osd;
}
