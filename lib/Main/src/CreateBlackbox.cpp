#include "Cockpit.h"
#include "Main.h"

#include <AHRS.h>
#include <AHRS_MessageQueue.h>
#include <BlackboxCallbacks.h>
#include <BlackboxProtoflight.h>
#include <BlackboxSerialDeviceSDCard.h>
#include <Debug.h>
#include <ReceiverBase.h>
#include <TimeMicroseconds.h>


/*!
Statically allocate the Blackbox and associated objects.
*/
Blackbox* Main::createBlackBox(AHRS& ahrs, FlightController& flightController, Cockpit& cockpit, const ReceiverBase& receiver, const IMU_Filters& imuFilters, const Debug& debug) // cppcheck-suppress constParameterReference 
{
#if defined(USE_BLACKBOX)
    static BlackboxCallbacks            blackboxCallbacks(flightController.getAHRS_MessageQueue(), ahrs, flightController, cockpit, receiver, debug);
    static BlackboxSerialDeviceSDCard   blackboxSerialDevice(BlackboxSerialDeviceSDCard::SDCARD_SPI_PINS);

    static BlackboxProtoflight          blackbox(flightController.getTaskIntervalMicroseconds(), blackboxCallbacks, blackboxSerialDevice, flightController, cockpit, imuFilters);
    cockpit.setBlackbox(blackbox);

    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        //.device = Blackbox::DEVICE_NONE,
        .mode = Blackbox::MODE_NORMAL // logging starts on arming, file is saved when disarmed
        //.mode = Blackbox::MODE_ALWAYS_ON
    });
#if defined(USE_BLACKBOX_DEBUG)
    testBlackbox(blackbox, ahrs, receiver, debug);
#endif
    return &blackbox;
#else
    (void)ahrs;
    (void)flightController;
    (void)cockpit;
    (void)receiver;
    (void)imuFilters;
    (void)debug;
    return nullptr;
#endif
}

#if defined(USE_BLACKBOX)
void Main::testBlackbox(Blackbox& blackbox, AHRS& ahrs, ReceiverBase& receiver, const Debug& debug)
{
    static uint32_t timeMicrosecondsPrevious = 0;

    print("***StartLog\r\n");
    blackbox.start(Blackbox::start_t{.debugMode = static_cast<uint16_t>(debug.getMode()), .motorCount = 4, .servoCount = 0});

    for (size_t ii = 0; ii < 1500; ++ii) {
        const uint32_t timeMicroseconds = timeUs();

        static const uint32_t timeMicrosecondsDelta = timeMicroseconds - timeMicrosecondsPrevious;
        timeMicrosecondsPrevious = timeMicroseconds;

        const uint32_t state = blackbox.updateLog(timeMicroseconds);
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