#include "Main.h"

#include <CMS.h>


/*!
Statically allocate the CMS.
*/
CMS& Main::createCMS(DisplayPortBase& displayPort, const ReceiverBase& receiver, Cockpit& cockpit, IMU_Filters& imuFilters, IMU_Base& imu, NonVolatileStorage& nvs, OSD* osd) // cppcheck-suppress constParameterReference
{
    static CMS cms(&displayPort, receiver, cockpit, imuFilters, imu, nvs, osd);

    return cms;
}
