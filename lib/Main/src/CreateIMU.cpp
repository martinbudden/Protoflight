#include "Main.h"

#include <IMU_BMI270.h>
#include <IMU_BNO085.h>
#include <IMU_ICM426xx.h>
#include <IMU_LSM6DS3TR_C.h>
#include <IMU_M5Unified.h>
#include <IMU_MPU6000.h>
#include <IMU_MPU6886.h>
#include <IMU_Null.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif


/*!
Create the IMU using the specified build flags.
Set the IMU target gyro sample rate to GYRO_SAMPLE_RATE_HZ.
*/
IMU_Base& Main::createIMU()
{
    // Statically allocate the IMU according the the build flags
// NOLINTBEGIN(misc-const-correctness) false positive
#if defined(LIBRARY_IMU_USE_SPI_BUS)
    enum { SPI_8_MEGAHERTZ = 8000000, SPI_10_MEGAHERTZ = 10000000, SPI_20_MEGAHERTZ = 20000000 };
#if defined(USE_IMU_MPU6886)
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, SPI_10_MEGAHERTZ, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#elif defined(USE_IMU_MPU6000)
    static IMU_MPU6000 imuSensor(IMU_AXIS_ORDER, SPI_20_MEGAHERTZ, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#elif defined(USE_IMU_BMI270)
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, SPI_20_MEGAHERTZ,  BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#elif defined(USE_IMU_BNO085)
    static IMU_BNO085 imuSensor(IMU_AXIS_ORDER, SPI_20_MEGAHERTZ, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#elif defined(USE_IMU_ICM20602)
    static IMU_ICM20602 imuSensor(IMU_AXIS_ORDER, SPI_10_MEGAHERTZ, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#elif defined(USE_IMU_ICM20608)
    static IMU_ICM20608 imuSensor(IMU_AXIS_ORDER, SPI_8_MEGAHERTZ, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#elif defined(USE_IMU_ICM426XX)
    static IMU_ICM426xx imuSensor(IMU_AXIS_ORDER, SPI_20_MEGAHERTZ, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#elif defined(USE_IMU_LSM6DS3TR_C) || defined(USE_IMU_ISM330DHCX) || defined(USE_LSM6DSOX)
    static IMU_LSM6DS3TR_C imuSensor(IMU_AXIS_ORDER, SPI_20_MEGAHERTZ, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#else
    static_assert(false);
#endif

#else

#if defined(USE_IMU_MPU6886)
#if defined(M5_UNIFIED)
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, BUS_I2C::pins_t{.sda=static_cast<uint8_t>(M5.In_I2C.getSDA()), .scl=static_cast<uint8_t>(M5.In_I2C.getSCL()), .irq=0xFF});
#else
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, BUS_I2C::IMU_I2C_PINS);
#endif
#elif defined(USE_IMU_MPU6000)
    static IMU_MPU6000 imuSensor(IMU_AXIS_ORDER, BUS_I2C::IMU_I2C_PINS);
#elif defined(USE_IMU_BMI270)
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, BUS_I2C::IMU_I2C_PINS);
#elif defined(USE_IMU_BNO085)
    static IMU_BNO085 imuSensor(IMU_AXIS_ORDER, BUS_I2C::IMU_I2C_PINS);
#elif defined(USE_IMU_ICM426XX)
    static IMU_ICM426xx imuSensor(IMU_AXIS_ORDER, BUS_I2C::IMU_I2C_PINS);
#elif defined(USE_IMU_LSM6DS3TR_C) || defined(USE_IMU_ISM330DHCX) || defined(USE_LSM6DSOX)
    static IMU_LSM6DS3TR_C imuSensor(IMU_AXIS_ORDER, BUS_I2C::IMU_I2C_PINS);
#elif defined(USE_IMU_M5_UNIFIED)
    static IMU_M5_UNIFIED imuSensor(IMU_AXIS_ORDER);
#elif defined(USE_IMU_NULL)
    static IMU_Null imuSensor;
#else
    static_assert(false);
#endif

#endif // LIBRARY_IMU_USE_SPI_BUS
// NOLINTEND(misc-const-correctness) false positive

    static_cast<IMU_Base&>(imuSensor).init(GYRO_SAMPLE_RATE_HZ);

    return imuSensor;
}
