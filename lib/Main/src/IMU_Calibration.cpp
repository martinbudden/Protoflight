#include "Main.h"

#include <AHRS.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#include "NonVolatileStorage.h"


void Main::calibrateIMUandSave(NonVolatileStorage& nvs, IMU_Base& imu, IMU_Base::calibration_type_e calibrationType)
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

    enum { CALIBRATION_COUNT = 5000 };
    imu.calibrate(calibrationType, CALIBRATION_COUNT);
    const xyz_t gyroOffset = imu.getGyroOffset();
    const xyz_t accOffset = imu.getAccOffset();
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
    if (calibrationType == IMU_Base::CALIBRATE_ACC_AND_GYRO) {
        nvs.storeAccOffset(accOffset);
        nvs.storeAccCalibrationState(NonVolatileStorage::CALIBRATED);
    }

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

void Main::checkIMU_Calibration(NonVolatileStorage& nvs, IMU_Base& imu) // cppcheck-suppress constParameterReference
{
    // Set the gyro offsets from non-volatile storage.
    if (nvs.loadGyroCalibrationState() == NonVolatileStorage::CALIBRATED) {
        const xyz_t gyroOffset = nvs.loadGyroOffset();
        imu.setGyroOffset(gyroOffset);
#if !defined(FRAMEWORK_STM32_CUBE)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** AHRS gyroOffsets loaded from NVS: gx:%f, gy:%f, gz:%f\r\n", static_cast<double>(gyroOffset.x), static_cast<double>(gyroOffset.y), static_cast<double>(gyroOffset.z));
        print(&buf[0]);
#endif
        if (nvs.loadAccCalibrationState() == NonVolatileStorage::CALIBRATED) {
            const xyz_t accOffset = nvs.loadGyroOffset();
            imu.setAccOffset(accOffset);
#if !defined(FRAMEWORK_STM32_CUBE)
            sprintf(&buf[0], "**** AHRS accOffsets loaded from NVS: ax:%f, ay:%f, az:%f\r\n", static_cast<double>(accOffset.x), static_cast<double>(accOffset.y), static_cast<double>(accOffset.z));
            print(&buf[0]);
#endif
        }
    } else {
        // when calibrateIMU called automatically on startup, just calibrate the gyroscope.
        //!!calibrateIMUandSave(nvs, imu, CALIBRATE_GYRO_ONLY);
    }
}
