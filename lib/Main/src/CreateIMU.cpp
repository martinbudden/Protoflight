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

#include <cstdio>


/*!
Create the IMU using the specified build flags.
Set the IMU target gyro sample rate to GYRO_SAMPLE_RATE_HZ.
*/
IMU_Base& Main::createIMU(NonVolatileStorage& nvs)
{
#if defined(OPTICAL_FLOW_PINS)
    // we need to deselect the optical flow chip, which is on the same SPI bus as the IMU.
    const BUS_SPI::spi_pins_t opticalFlowPins = BUS_SPI::OPTICAL_FLOW_PINS;
#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(FRAMEWORK_ESPIDF)
    const gpio_config_t opticalFlowConfig = {
        .pin_bit_mask = (1ULL << opticalFlowPins.cs),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&opticalFlowConfig);
    gpio_set_level(static_cast<gpio_num_t>(opticalFlowPins.cs), 1);
#else
    pinMode(opticalFlowPins.cs, OUTPUT);
    digitalWrite(opticalFlowPins.cs, 1);
#endif
#endif // OPTICAL_FLOW_PINS

    // Statically allocate the IMU according the the build flags
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    enum { SPI_8_MEGAHERTZ = 8000000, SPI_10_MEGAHERTZ = 10000000, SPI_20_MEGAHERTZ = 20000000 };
#if defined(USE_IMU_MPU6886)
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, SPI_10_MEGAHERTZ, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#elif defined(USE_IMU_MPU6000)
    static IMU_MPU6000 imuSensor(IMU_AXIS_ORDER, SPI_20_MEGAHERTZ, BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#elif defined(USE_IMU_BMI270)
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, SPI_10_MEGAHERTZ,  BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
    //imuSensor.init(GYRO_SAMPLE_RATE_HZ, IMU_Base::GYRO_FULL_SCALE_MAX, IMU_Base::ACC_FULL_SCALE_MAX, nullptr);
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
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, BUS_I2C::i2c_pins_t{.sda=static_cast<uint8_t>(M5.In_I2C.getSDA()), .scl=static_cast<uint8_t>(M5.In_I2C.getSCL()), .irq=0xFF});
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
    //static IMU_M5_UNIFIED imuSensor(IMU_Base::YPOS_XPOS_ZNEG);
#elif defined(USE_IMU_NULL)
    static IMU_Null imuSensor;
#else
    static_assert(false);
#endif

#endif // LIBRARY_SENSORS_IMU_USE_SPI_BUS

    static_cast<IMU_Base&>(imuSensor).init(GYRO_SAMPLE_RATE_HZ);

#if defined(M5_UNIFIED)
    // Holding BtnA down while switching on enters calibration mode.
    if (M5.BtnA.isPressed()) {
        calibrateIMUandSave(nvs, imuSensor, CALIBRATE_ACC_AND_GYRO);
    }
#endif
    checkIMU_Calibration(nvs, imuSensor);

    const uint32_t imuSampleRateHz = imuSensor.getGyroSampleRateHz();
    std::array<char, 128> buf;
    sprintf(&buf[0], "\r\n**** IMU sample rate:%dHz\r\n\r\n", static_cast<int>(imuSampleRateHz));
    print(&buf[0]);

    return imuSensor;
}
