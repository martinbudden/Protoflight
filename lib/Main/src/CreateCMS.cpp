#include "Main.h"

#include <CMS.h>


/*!
Statically allocate the CMS.
*/
CMS* Main::createCMS(DisplayPortBase& displayPort, const ReceiverBase& receiver, Cockpit& cockpit, IMU_Filters& imuFilters, IMU_Base& imu, NonVolatileStorage& nvs, OSD* osd, VTX_Base* vtx) // cppcheck-suppress constParameterReference
{
#if defined(USE_CMS)
    static CMS cms(&displayPort, receiver, cockpit, imuFilters, imu, nvs, osd, vtx);

    return &cms;
#else
    (void)displayPort;
    (void)receiver;
    (void)cockpit;
    (void)imuFilters;
    (void)imu;
    (void)nvs;
    (void)osd;
    (void)vtx;

    return nullptr;
#endif
}
