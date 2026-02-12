#include "Main.h"

#include <CMS.h>


/*!
Statically allocate the CMS.
*/
CMS* Main::createCMS(DisplayPortBase& displayPort, Cockpit& cockpit, const ReceiverBase& receiver, RcModes& rc_modes, IMU_Filters& imuFilters, IMU_Base& imu, NonVolatileStorage& nvs, VTX* vtx) // cppcheck-suppress constParameterReference
{
#if defined(USE_CMS)
    static CMS cms(&displayPort, cockpit, receiver, rc_modes, imuFilters, imu, nvs, vtx);

    return &cms;
#else
    (void)displayPort;
    (void)cockpit;
    (void)receiver;
    (void)rc_modes;
    (void)nvs;
    (void)imuFilters;
    (void)imu;
    (void)vtx;

    return nullptr;
#endif
}
