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
    const xyz_t gyroOffset = xyz_t {
        .x = static_cast<float>(gyroX) / count,
        .y = static_cast<float>(gyroY) / count,
        .z = static_cast<float>(gyroZ) / count
    } *  ahrs.getIMU().getAccResolution();

    xyz_t accOffset = {
        .x = static_cast<float>(accX) / count,
        .y = static_cast<float>(accY) / count,
        .z = static_cast<float>(accZ) / count
    };

    const float oneG = 1.0F / ahrs.getIMU().getAccResolution();
    const float halfG = oneG / 2.0F;
    if (accOffset.x > halfG) {
        accOffset.x -= oneG;
    } else if (accOffset.x < - halfG) {
        accOffset.x += oneG;
    } else if (accOffset.y > halfG) {
        accOffset.y -= oneG;
    } else if (accOffset.y < - halfG) {
        accOffset.y += oneG;
    } else if (accOffset.z > halfG) {
        accOffset.z -= oneG;
    } else if (accOffset.z < - halfG) {
        accOffset.z += oneG;
    }
    accOffset *= ahrs.getIMU().getAccResolution();

#if defined(M5_UNIFIED)
    if (M5.Lcd.width() > 300) {
        M5.Lcd.printf("gyro offsets\r\n");
        M5.Lcd.printf("x:%f y:%f z:%f\r\n\r\n", static_cast<double>(gyroOffset.x), static_cast<double>(gyroOffset.y), static_cast<double>(gyroOffset.z));
        M5.Lcd.printf("acc offsets\r\n");
        M5.Lcd.printf("x:%f y:%f z:%f\r\n\r\n", static_cast<double>(accOffset.x), static_cast<double>(accOffset.y), static_cast<double>(accOffset.z));
    }
#endif

    nvs.storeGyroOffset(gyroOffset);
    nvs.storeGyroCalibrationState(NonVolatileStorage::CALIBRATED);
    if (calibrationType == CALIBRATE_ACC_AND_GYRO) {
        nvs.storeAccOffset(accOffset);
        nvs.storeAccCalibrationState(NonVolatileStorage::CALIBRATED);
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
    if (nvs.loadGyroCalibrationState() == NonVolatileStorage::CALIBRATED) {
        const xyz_t gyroOffset = nvs.loadGyroOffset();
        ahrs.setGyroOffset(gyroOffset);
#if !defined(FRAMEWORK_STM32_CUBE)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** AHRS gyroOffsets loaded from NVS: gx:%f, gy:%f, gz:%f\r\n", static_cast<double>(gyroOffset.x), static_cast<double>(gyroOffset.y), static_cast<double>(gyroOffset.z));
        print(&buf[0]);
#endif
        if (nvs.loadAccCalibrationState() == NonVolatileStorage::CALIBRATED) {
            const xyz_t accOffset = nvs.loadGyroOffset();
            ahrs.setAccOffset(accOffset);
#if !defined(FRAMEWORK_STM32_CUBE)
            sprintf(&buf[0], "**** AHRS accOffsets loaded from NVS: ax:%f, ay:%f, az:%f\r\n", static_cast<double>(accOffset.x), static_cast<double>(accOffset.y), static_cast<double>(accOffset.z));
            print(&buf[0]);
#endif
        }
    } else {
        // when calibrateIMU called automatically on startup, just calibrate the gyroscope.
        //!!calibrateIMU(nvs, ahrs, CALIBRATE_GYRO_ONLY);
    }
}
