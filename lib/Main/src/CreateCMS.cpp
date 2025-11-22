#include "Main.h"

#include <CMS.h>


/*!
Statically allocate the CMS.
*/
CMS& Main::createCMS(DisplayPortBase& displayPort, const ReceiverBase& receiver, Cockpit& cockpit, IMU_Filters& imuFilters, OSD* osd) // cppcheck-suppress constParameterReference
{
    static CMS cms(&displayPort, receiver, cockpit, imuFilters, osd);

    return cms;
}
