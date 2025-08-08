#include "Main.h"

#include <AHRS.h>
#include <SensorFusion.h>

AHRS& Main::createAHRS(uint32_t AHRS_taskIntervalMicroSeconds, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters)
{
    // Statically allocate the Sensor Fusion Filter
    // Timings are for 240MHz ESP32-S3
#if defined(USE_COMPLEMENTARY_FILTER)
    // approx 130 microseconds per update
    static ComplementaryFilter sensorFusionFilter;
#elif defined(USE_MAHONY_FILTER)
    // approx 10 microseconds per update
    static MahonyFilter sensorFusionFilter;
#elif defined(USE_VQF)
    const float deltaT = static_cast<float>(AHRS_taskIntervalMicroSeconds) / 1000000.0F;
    static VQF sensorFusionFilter(deltaT, deltaT, deltaT, true, false, false);
#elif defined(USE_VQF_BASIC)
    static BasicVQF sensorFusionFilter(static_cast<float>(AHRS_taskIntervalMicroSeconds) / 1000000.0F);
#else
    // approx 16 microseconds per update
    static MadgwickFilter sensorFusionFilter;
#endif
// NOLINTEND(misc-const-correctness)

    // Statically allocate the AHRS object
    static AHRS ahrs(AHRS_taskIntervalMicroSeconds, sensorFusionFilter, imuSensor, imuFilters);
    return ahrs;
}
