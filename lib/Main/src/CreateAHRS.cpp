#include "Main.h"

#include <AHRS.h>
#include <IMU_BMI270.h>
#include <IMU_BNO085.h>
#include <IMU_ICM426xx.h>
#include <IMU_LSM6DS3TR_C.h>
#include <IMU_M5Unified.h>
#include <IMU_MPU6000.h>
#include <IMU_MPU6886.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <SensorFusion.h>

AHRS& Main::createAHRS(uint32_t AHRS_taskIntervalMicroSeconds, IMU_FiltersBase& imuFilters)
{
    // Statically allocate the IMU according the the build flags
// NOLINTBEGIN(misc-const-correctness) false positive
    [[maybe_unused]] static const uint32_t spiFrequency = 20000000;
#if defined(USE_IMU_MPU6886_I2C)
#if defined(M5_UNIFIED)
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, BUS_I2C::pins_t{.sda=static_cast<uint8_t>(M5.In_I2C.getSDA()), .scl=static_cast<uint8_t>(M5.In_I2C.getSCL()), .irq=0xFF, .irqLevel=0});
#else
    const BUS_I2C::pins_t pins = IMU_I2C_PINS;
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, pins);
#endif
#elif defined(USE_IMU_MPU6886_SPI)
    const BUS_SPI::pins_t pins = IMU_SPI_PINS;
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_MPU6000_I2C)
    const BUS_I2C::pins_t pins = IMU_I2C_PINS;
    static IMU_MPU6000 imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_MPU6000_SPI)
    const BUS_SPI::pins_t pins = IMU_SPI_PINS;
    static IMU_MPU6000 imuSensor(IMU_AXIS_ORDER, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_BMI270_I2C)
    const BUS_I2C::pins_t pins = IMU_I2C_PINS;
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_BMI270_SPI)
    const BUS_SPI::pins_t pins = IMU_SPI_PINS;
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_BNO085_I2C)
    const BUS_I2C::pins_t pins = IMU_I2C_PINS;
    static IMU_BNO085 imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_BNO085_SPI)
    const BUS_SPI::pins_t pins = IMU_SPI_PINS;
    static IMU_BNO085 imuSensor(IMU_AXIS_ORDER, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_ICM426XX_I2C)
    const BUS_I2C::pins_t pins = IMU_I2C_PINS;
    static IMU_ICM426xx imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_ICM426XX_SPI)
    const BUS_SPI::pins_t pins = IMU_SPI_PINS;
    static IMU_ICM426xx imuSensor(IMU_AXIS_ORDER, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_LSM6DS3TR_C_I2C) || defined(USE_IMU_ISM330DHCX_I2C) || defined(USE_LSM6DSOX_I2C)
    const BUS_I2C::pins_t pins = IMU_I2C_PINS;
    static IMU_LSM6DS3TR_C imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_LSM6DS3TR_C_SPI) || defined(USE_IMU_ISM330DHCX_SPI) || defined(USE_LSM6DSOX_SPI)
    const BUS_SPI::pins_t pins = IMU_SPI_PINS;
    static IMU_LSM6DS3TR_C imuSensor(IMU_AXIS_ORDER, spiFrequency, BUS_SPI::SPI_INDEX_0, pins);
#elif defined(USE_IMU_M5_UNIFIED)
    static IMU_M5_UNIFIED imuSensor(IMU_AXIS_ORDER);
#else
    static_assert(false);
#endif

    //static_cast<IMU_Base&>(imuSensor).init(1000000 / AHRS_taskIntervalMicroSeconds);
    static_cast<IMU_Base&>(imuSensor).init();

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

