#include "cockpit.h"
#include "flight_controller.h"
#include "imu_filters.h"
#include "main.h"

#include <blackbox_callbacks.h>
#include <blackbox_protoflight.h>

#include <ahrs.h>
#include <blackbox_serial_device_sdcard.h>
#include <debug.h>
#include <receiver_base.h>
#include <time_microseconds.h>


/*!
Statically allocate the Blackbox and associated objects.
*/
Blackbox* Main::create_blackbox(uint32_t task_interval_microseconds) // cppcheck-suppress constParameterReference
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
        .fields_disabled_mask = 0,
    });
    return &blackbox;
#else
    (void)task_interval_microseconds;
    return nullptr;
#endif
}

#if defined(USE_BLACKBOX)
void Main::test_blackbox(Blackbox& blackbox, Ahrs& ahrs, ReceiverBase& receiver, FlightController& flight_controller, ImuFilters& imu_filters, Debug& debug, const blackbox_context_t& ctx)
{
    static uint32_t time_microseconds_previous = 0;

    print("***StartLog\r\n");
    blackbox.start(Blackbox::start_t{.debug_mode = debug.get_mode(), .motor_count = 4, .servo_count = 0});

    for (size_t ii = 0; ii < 1500; ++ii) {
        const uint32_t time_microseconds = time_us();

        static const uint32_t time_microseconds_delta = time_microseconds - time_microseconds_previous;
        time_microseconds_previous = time_microseconds;

        const uint32_t state = blackbox.update_log(ctx, time_microseconds);
        (void)state;
        // Serial.printf("ii:%3d, s:%2d\r\n", ii, state);

        ahrs.read_imu_and_update_orientation(time_microseconds, time_microseconds_delta, imu_filters, flight_controller, debug);
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