#include "Main.h"

#include <AHRS.h>
#include <BlackboxCallbacks.h>
#include <BlackboxMessageQueueAHRS.h>
#include <BlackboxProtoFlight.h>
#include <BlackboxSerialDeviceSDCard.h>
#include <Debug.h>
#include <RadioController.h>
#include <TimeMicroseconds.h>


#if defined(USE_BLACKBOX)
/*!
Statically allocate the Blackbox and associated objects.
*/
Blackbox& Main::createBlackBox(AHRS& ahrs, FlightController& flightController, RadioController& radioController, IMU_Filters& imuFilters, Debug& debug)
{
    ReceiverBase& receiver = const_cast<ReceiverBase&>(radioController.getReceiver()); // NOLINT(cppcoreguidelines-pro-type-const-cast,hicpp-use-auto,modernize-use-auto)

    // Statically allocate the Blackbox and associated objects
    static BlackboxMessageQueue         blackboxMessageQueue;
    static BlackboxCallbacks            blackboxCallbacks(blackboxMessageQueue, ahrs, flightController, radioController, receiver, debug);
    static BlackboxSerialDeviceSDCard   blackboxSerialDevice(BlackboxSerialDeviceSDCard::SDCARD_SPI_PINS); // NOLINT(misc-const-correctness)
    static BlackboxProtoFlight          blackbox(blackboxCallbacks, blackboxMessageQueue, blackboxSerialDevice, flightController, radioController, imuFilters);

    static BlackboxMessageQueueAHRS     blackboxMessageQueueAHRS(blackboxMessageQueue);
    ahrs.setMessageQueue(&blackboxMessageQueueAHRS);

    flightController.setBlackbox(blackbox);
    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        //.device = Blackbox::DEVICE_NONE,
        .mode = Blackbox::MODE_NORMAL // logging starts on arming, file is saved when disarmed
        //.mode = Blackbox::MODE_ALWAYS_ON
    });
#if defined(USE_BLACKBOX_DEBUG)
    testBlackbox(blackbox, ahrs, receiver, debug);
#else
    blackboxCallbacks.setUseMessageQueue(true);
#endif
    return blackbox;
}


void Main::testBlackbox(Blackbox& blackbox, AHRS& ahrs, ReceiverBase& receiver, const Debug& debug)
{
    static uint32_t timeMicrosecondsPrevious = 0;

    print("***StartLog\r\n");
    blackbox.start(Blackbox::start_t{.debugMode = static_cast<uint16_t>(debug.getMode()), .motorCount = 4, .servoCount = 0});

    for (size_t ii = 0; ii < 1500; ++ii) {
        const uint32_t timeMicroseconds = timeUs();

        static const uint32_t timeMicrosecondsDelta = timeMicroseconds - timeMicrosecondsPrevious;
        timeMicrosecondsPrevious = timeMicroseconds;

        const uint32_t state = blackbox.update(timeMicroseconds);
        (void)state;
        // Serial.printf("ii:%3d, s:%2d\r\n", ii, state);

        ahrs.readIMUandUpdateOrientation(timeMicroseconds, timeMicrosecondsDelta);
        receiver.update(timeMicrosecondsDelta / 1000);
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