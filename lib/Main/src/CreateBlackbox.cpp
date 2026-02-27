#include "Cockpit.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "Main.h"

#include <BlackboxCallbacks.h>
#include <BlackboxProtoflight.h>

#include <ahrs.h>
#include <blackbox_serial_device_sdcard.h>
#include <debug.h>
#include <receiver_base.h>
#include <time_microseconds.h>


/*!
Statically allocate the Blackbox and associated objects.
*/
Blackbox* Main::createBlackBox(uint32_t task_interval_microseconds) // cppcheck-suppress constParameterReference
{
#if defined(USE_BLACKBOX)
    static BlackboxCallbacks            blackboxCallbacks;
    static BlackboxSerialDeviceSDCard   blackboxSerialDevice(BlackboxSerialDeviceSDCard::SDCARD_SPI_PINS);

    static BlackboxProtoflight          blackbox(task_interval_microseconds, blackboxCallbacks, blackboxSerialDevice);

    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        //.device = Blackbox::DEVICE_NONE,
        .mode = Blackbox::MODE_NORMAL, // logging starts on arming, file is saved when disarmed
        //.mode = Blackbox::MODE_ALWAYS_ON,
        .gps_use_3d_speed = false,
        .fieldsDisabledMask = 0,
    });
    return &blackbox;
#else
    (void)task_interval_microseconds;
    return nullptr;
#endif
}

#if defined(USE_BLACKBOX)
void Main::testBlackbox(Blackbox& blackbox, Ahrs& ahrs, ReceiverBase& receiver, FlightController& flightController, IMU_Filters& imuFilters, Debug& debug, const blackbox_parameter_group_t& pg)
{
    static uint32_t time_microseconds_previous = 0;

    print("***StartLog\r\n");
    blackbox.start(Blackbox::start_t{.debugMode = static_cast<uint16_t>(debug.getMode()), .motorCount = 4, .servoCount = 0});

    for (size_t ii = 0; ii < 1500; ++ii) {
        const uint32_t time_microseconds = time_us();

        static const uint32_t time_microseconds_delta = time_microseconds - time_microseconds_previous;
        time_microseconds_previous = time_microseconds;

        const uint32_t state = blackbox.update_log(pg, time_microseconds);
        (void)state;
        // Serial.printf("ii:%3d, s:%2d\r\n", ii, state);

        ahrs.read_imu_and_update_orientation(time_microseconds, time_microseconds_delta, imuFilters, flightController, debug);
        receiver.update(time_microseconds_delta / 1000);
#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else
        delay(1);
#endif
    }

    //blackboxSerialDevice.endLog(true);
    blackbox.finish();
    print("***EndLog\r\n");
#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else
    delay(5000);
#endif
}
#endif