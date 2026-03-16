#include "main.h"

#include <ahrs.h>
#include <sensor_fusion.h>


Ahrs& Main::create_ahrs(ImuBase& imuSensor)
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
    const uint32_t imuSample_rate_hz = imuSensor.getGyroSample_rate_hz();
    const float delta_t = 1.0F / static_cast<float>(imuSample_rate_hz);
    static VQF sensorFusionFilter(delta_t, delta_t, delta_t, true, false, false);
#elif defined(USE_VQF_BASIC)
    const uint32_t imuSample_rate_hz = imuSensor.getGyroSample_rate_hz();
    const float delta_t = 1.0F / static_cast<float>(imuSample_rate_hz);
    static BasicVQF sensorFusionFilter(delta_t);
#else
    // approx 16 microseconds per update
    static MadgwickFilter sensorFusionFilter;
#endif

    // Statically allocate the AHRS object
#if defined(AHRS_TASK_IS_TIMER_DRIVEN)
    static Ahrs ahrs(Ahrs::TIMER_DRIVEN, sensorFusionFilter, imuSensor);
#else
    static Ahrs ahrs(Ahrs::INTERRUPT_DRIVEN, sensorFusionFilter, imuSensor);
#endif
    return ahrs;
}
