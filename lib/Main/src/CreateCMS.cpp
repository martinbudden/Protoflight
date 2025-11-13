#include "Main.h"

#include <CMS.h>


/*!
Statically allocate the CMS.
*/
CMS& Main::createCMS(OSD& osd, const ReceiverBase& receiver, const FlightController& flightController, Cockpit& cockpit) // cppcheck-suppress constParameterReference
{
    static CMS cms(osd, receiver, flightController, cockpit);

    return cms;
}
