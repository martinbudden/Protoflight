#include "Main.h"

#include <CMS.h>


/*!
Statically allocate the CMS.
*/
CMS* Main::createCMS(DisplayPortBase& displayPort, const ReceiverBase& receiver, Cockpit& cockpit, IMU_Filters& imuFilters, IMU_Base& imu, OSD* osd, VTX* vtx) // cppcheck-suppress constParameterReference
{
#if defined(USE_CMS)
    static CMS cms(&displayPort, receiver, cockpit, imuFilters, imu, osd, vtx);

    return &cms;
#else
    (void)displayPort;
    (void)receiver;
    (void)cockpit;
    (void)imuFilters;
    (void)imu;
    (void)osd;
    (void)vtx;

    return nullptr;
#endif
}
