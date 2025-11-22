#include "IMU_Filters.h"
#include "Main.h"
#include "MotorMixerBase.h"
#include "NonVolatileStorage.h"

#include <RPM_Filters.h>


IMU_Filters& Main::createIMU_Filters(float taskIntervalSeconds, MotorMixerBase& motorMixer, Debug& debug, const NonVolatileStorage& nvs)
{
    // Statically allocate the IMU_Filters
    static IMU_Filters imuFilters(motorMixer.getMotorCount(), debug, taskIntervalSeconds);
    imuFilters.setConfig(nvs.loadIMU_FiltersConfig());
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    imuFilters.setDynamicNotchFilterConfig(nvs.loadDynamicNotchFilterConfig());
#endif
#if defined(USE_RPM_FILTERS)
    RPM_Filters* rpmFilters = motorMixer.getRPM_Filters();
    if (rpmFilters) {
        rpmFilters->setConfig(nvs.loadRPM_FiltersConfig());
        imuFilters.setRPM_Filters(rpmFilters);
    }
#endif
    return imuFilters;
}
