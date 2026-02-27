#include "Main.h"
#include "NonVolatileStorage.h"

#include <IMU_Base.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#include <ahrs.h>


void Main::calibrateIMUandSave(NonVolatileStorage& nvs, ImuBase& imu, calibration_type_e calibrationType)
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
    ImuBase::delay_ms(4000); // delay 4 seconds to allow robot to stabilize after user lets go

    enum { CALIBRATION_COUNT = 5000 };
    imu.calibrate(calibrationType == CALIBRATE_ACC_AND_GYRO ? ImuBase::CALIBRATE_ACC_AND_GYRO : ImuBase::CALIBRATE_GYRO_ONLY, CALIBRATION_COUNT);
    const xyz_t gyro_offset = imu.get_gyro_offset();
    const xyz_t acc_offset = imu.get_acc_offset();
#if defined(M5_UNIFIED)
    if (M5.Lcd.width() > 300) {
        M5.Lcd.printf("gyro offsets\r\n");

        M5.Lcd.printf("x:%f y:%f z:%f\r\n\r\n", static_cast<double>(gyro_offset.x), static_cast<double>(gyro_offset.y), static_cast<double>(gyro_offset.z));
        M5.Lcd.printf("acc offsets\r\n");
        M5.Lcd.printf("x:%f y:%f z:%f\r\n\r\n", static_cast<double>(acc_offset.x), static_cast<double>(acc_offset.y), static_cast<double>(acc_offset.z));
    }
#endif
    nvs.store_gyro_offset(gyro_offset);
    nvs.store_gyro_calibration_state(NonVolatileStorage::CALIBRATED);
    if (calibrationType == CALIBRATE_ACC_AND_GYRO) {
        nvs.store_acc_offset(acc_offset);
        nvs.store_acc_calibration_state(NonVolatileStorage::CALIBRATED);
    }

#if defined(M5_UNIFIED)
    M5.Lcd.printf("Finished calibration\r\n");
#endif

    ImuBase::delay_ms(4000); // delay 4 seconds to allow user to read screen

#if defined(M5_UNIFIED)
    //M5.Power.powerOff();
    M5.Power.timerSleep(0); // sleep for zero seconds and reboot
#endif
}

void Main::checkIMU_Calibration(NonVolatileStorage& nvs, ImuBase& imu) // cppcheck-suppress constParameterReference
{
    // Set the gyro offsets from non-volatile storage.
    if (nvs.load_gyro_calibration_state() == NonVolatileStorage::CALIBRATED) {
        const xyz_t gyro_offset = nvs.load_gyro_offset();
        imu.set_gyro_offset(gyro_offset);
#if !defined(FRAMEWORK_STM32_CUBE)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** AHRS gyro_offsets loaded from NVS: gx:%f, gy:%f, gz:%f\r\n", static_cast<double>(gyro_offset.x), static_cast<double>(gyro_offset.y), static_cast<double>(gyro_offset.z));
        print(&buf[0]);
#endif
        if (nvs.load_acc_calibration_state() == NonVolatileStorage::CALIBRATED) {
            const xyz_t acc_offset = nvs.load_gyro_offset();
            imu.set_acc_offset(acc_offset);
#if !defined(FRAMEWORK_STM32_CUBE)
            sprintf(&buf[0], "**** AHRS acc_offsets  loaded from NVS: ax:%f, ay:%f, az:%f\r\n", static_cast<double>(acc_offset.x), static_cast<double>(acc_offset.y), static_cast<double>(acc_offset.z));
            print(&buf[0]);
#endif
        }
    } else {
        // when calibrateIMU called automatically on startup, just calibrate the gyroscope.
        //!!calibrateIMUandSave(nvs, imu, CALIBRATE_GYRO_ONLY);
    }
}
