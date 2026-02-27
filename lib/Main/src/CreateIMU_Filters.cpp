#include "IMU_Filters.h"
#include "Main.h"
#include "NonVolatileStorage.h"

#include <motor_mixer_base.h>


IMU_Filters& Main::createIMU_Filters(float taskIntervalSeconds, RpmFilters* rpmFilters, const NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
    (void)rpmFilters;

    // Statically allocate the IMU_Filters
    static IMU_Filters imuFilters(taskIntervalSeconds);
    imuFilters.setConfig(nvs.load_imu_filters_config());
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    imuFilters.set_dynamic_notch_filter_config(nvs.load_dynamic_notch_filter_config());
#endif
#if defined(USE_RPM_FILTERS)
    imuFilters.setRPM_Filters(rpmFilters);
#endif
    return imuFilters;
}
