#include "Main.h"

#include <CMS.h>


/*!
Statically allocate the CMS.
*/
CMS* Main::createCMS(DisplayPortBase& displayPort, Cockpit& cockpit, const ReceiverBase& receiver, IMU_Filters& imuFilters, IMU_Base& imu, VTX* vtx) // cppcheck-suppress constParameterReference
{
#if defined(USE_CMS)
    static CMS cms(&displayPort, cockpit, receiver, imuFilters, imu, vtx);

    return &cms;
#else
    (void)displayPort;
    (void)cockpit;
    (void)receiver;
    (void)imuFilters;
    (void)imu;
    (void)vtx;

    return nullptr;
#endif
}
