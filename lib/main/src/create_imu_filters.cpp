#include "imu_filters.h"
#include "main.h"
#include "non_volatile_storage.h"

#include <motor_mixer_base.h>


ImuFilters& Main::create_imu_filters(float task_interval_seconds, RpmFilters* rpm_filters, const NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
    (void)rpm_filters;

    // Statically allocate the ImuFilters
    static ImuFilters imu_filters(task_interval_seconds);
    imu_filters.set_config(nvs.load_imu_filters_config());
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    imu_filters.set_dynamic_notch_filter_config(nvs.load_dynamic_notch_filter_config());
#endif
#if defined(USE_RPM_FILTERS)
    imu_filters.set_rpm_filters(rpm_filters);
#endif
    return imu_filters;
}
