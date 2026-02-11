#include "IMU_Filters.h"
#include "Main.h"
#include "NonVolatileStorage.h"

#include <MotorMixerBase.h>


IMU_Filters& Main::createIMU_Filters(float taskIntervalSeconds, MotorMixerBase& motorMixer, Debug& debug, const NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
    // Statically allocate the IMU_Filters
    static IMU_Filters imuFilters(motorMixer.get_motor_count(), debug, taskIntervalSeconds);
    imuFilters.setConfig(nvs.loadIMU_FiltersConfig());
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    imuFilters.setDynamicNotchFilterConfig(nvs.loadDynamicNotchFilterConfig());
#endif
#if defined(USE_RPM_FILTERS)
    RpmFilters* rpmFilters = motorMixer.get_rpm_filters();
    if (rpmFilters) {
        rpmFilters->set_config(nvs.loadRPM_FiltersConfig());
        imuFilters.setRPM_Filters(rpmFilters);
    }
#endif
    return imuFilters;
}
