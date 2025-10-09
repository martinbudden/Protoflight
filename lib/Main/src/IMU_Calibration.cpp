#include "Main.h"

#include <AHRS.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#include <NonVolatileStorage.h>


void Main::runIMU_Calibration(NonVolatileStorage& nvs, AHRS& ahrs, calibration_type_e calibrationType)
{
    int64_t gyroX = 0;
    int64_t gyroY = 0;
    int64_t gyroZ = 0;
    int64_t accX = 0;
    int64_t accY = 0;
    int64_t accZ = 0;

    int32_t x {};
    int32_t y {};
    int32_t z {};

    const int count = 5000;
    for (auto ii = 0; ii < count; ++ii) {
#if defined(FRAMEWORK_ARDUINO)
        delay(1);
#endif
        ahrs.readGyroRaw(x, y, z);
        gyroX += x;
        gyroY += y;
        gyroZ += z;
        ahrs.readAccRaw(x, y, z);
        accX += x;
        accY += y;
        accZ += z;
    }

    const auto gyroOffset_x = static_cast<int32_t>(gyroX / count);
    const auto gyroOffset_y = static_cast<int32_t>(gyroY / count);
    const auto gyroOffset_z = static_cast<int32_t>(gyroZ / count);

    auto accOffset_x = static_cast<int32_t>(accX / count);
    auto accOffset_y = static_cast<int32_t>(accY / count);
    auto accOffset_z = static_cast<int32_t>(accZ / count);

    const int32_t oneG = ahrs.getAccOneG_Raw();
    const int32_t halfG = oneG / 2;
    if (accOffset_x > halfG) {
        accOffset_x -= oneG;
    } else if (accOffset_x < - halfG) {
        accOffset_x += oneG;
    } else if (accOffset_y > halfG) {
        accOffset_y -= oneG;
    } else if (accOffset_y < - halfG) {
        accOffset_y += oneG;
    } else if (accOffset_z > halfG) {
        accOffset_z -= oneG;
    } else if (accOffset_z < - halfG) {
        accOffset_z += oneG;
    }

#if defined(M5_UNIFIED)
    if (M5.Lcd.width() > 300) {
        M5.Lcd.printf("gyro offsets\r\n");
        M5.Lcd.printf("x:%5d y:%5d z:%5d\r\n\r\n", static_cast<int>(gyroOffset_x), static_cast<int>(gyroOffset_y), static_cast<int>(gyroOffset_z));
        M5.Lcd.printf("acc offsets\r\n");
        M5.Lcd.printf("x:%5d y:%5d z:%5d\r\n\r\n", static_cast<int>(accOffset_x), static_cast<int>(accOffset_y), static_cast<int>(accOffset_z));
    }
#endif

    nvs.storeGyroOffset(gyroOffset_x, gyroOffset_y, gyroOffset_z);
    if (calibrationType == CALIBRATE_ACC_AND_GYRO) {
        nvs.storeAccOffset(accOffset_x, accOffset_y, accOffset_z);
    }
}

void Main::calibrateIMU(NonVolatileStorage& nvs, AHRS& ahrs, calibration_type_e calibrationType)
{
#if defined(M5_UNIFIED)
    if (M5.Lcd.width() > 300) {
        M5.Lcd.setTextSize(2);
    }
    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.fillScreen(TFT_BLACK);

    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("Starting gyro calibration\r\n");
    M5.Lcd.printf("Please keep the robot\r\n");
    M5.Lcd.printf("still for 10 seconds\r\n\r\n");
#endif
#if defined(FRAMEWORK_ARDUINO)
    delay(4000); // delay 4 seconds to allow robot to stabilize after user lets go
#endif

    runIMU_Calibration(nvs, ahrs, calibrationType);

#if defined(M5_UNIFIED)
    M5.Lcd.printf("Finished calibration\r\n");
#endif

#if defined(FRAMEWORK_ARDUINO)
    delay(4000); // delay 4 seconds to allow user to read screen
#endif

#if defined(M5_UNIFIED)
    //M5.Power.powerOff();
    M5.Power.timerSleep(0); // sleep for zero seconds and reboot
#endif
}

void Main::checkIMU_Calibration(NonVolatileStorage& nvs, AHRS& ahrs) // cppcheck-suppress constParameterReference
{
    // Set the gyro offsets from non-volatile storage.
    IMU_Base::xyz_int32_t offset {};
    if (nvs.loadGyroOffset(offset.x, offset.y, offset.z)) {
        ahrs.setGyroOffset(offset);
#if !defined(FRAMEWORK_STM32_CUBE)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** AHRS gyroOffsets loaded from NVS: gx:%5d, gy:%5d, gz:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
        print(&buf[0]);
#endif
        if (nvs.loadAccOffset(offset.x, offset.y, offset.z)) {
            ahrs.setAccOffset(offset);
#if !defined(FRAMEWORK_STM32_CUBE)
            sprintf(&buf[0], "**** AHRS accOffsets loaded from NVS: ax:%5d, ay:%5d, az:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
            print(&buf[0]);
#endif
        }
    } else {
        // when calibrateGyro called automatically on startup, just calibrate the gyroscope.
        //!!calibrateIMU(nonVolatileStorage, ahrs, CALIBRATE_GYRO_ONLY);
    }
}
