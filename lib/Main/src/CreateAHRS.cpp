#include "Main.h"

#include <AHRS.h>
#include <SensorFusion.h>


AHRS& Main::createAHRS(IMU_Base& imuSensor, IMU_FiltersBase& imuFilters)
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
    const uint32_t imuSampleRateHz = imuSensor.getGyroSampleRateHz();
    const float deltaT = 1.0F / static_cast<float>(imuSampleRateHz);
    static VQF sensorFusionFilter(deltaT, deltaT, deltaT, true, false, false);
#elif defined(USE_VQF_BASIC)
    const uint32_t imuSampleRateHz = imuSensor.getGyroSampleRateHz();
    const float deltaT = 1.0F / static_cast<float>(imuSampleRateHz);
    static BasicVQF sensorFusionFilter(deltaT);
#else
    // approx 16 microseconds per update
    static MadgwickFilter sensorFusionFilter;
#endif

    // Statically allocate the AHRS object
#if defined(AHRS_TASK_IS_TIMER_DRIVEN)
    static AHRS ahrs(AHRS::TIMER_DRIVEN, sensorFusionFilter, imuSensor, imuFilters);
#else
    static AHRS ahrs(AHRS::INTERRUPT_DRIVEN, sensorFusionFilter, imuSensor, imuFilters);
#endif
    return ahrs;
}
