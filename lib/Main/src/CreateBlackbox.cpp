
#include "Main.h"

#include <AHRS.h>
#include <BlackboxCallbacks.h>
#include <BlackboxMessageQueueAHRS.h>
#include <BlackboxProtoFlight.h>
#include <BlackboxSerialDeviceSDCard.h>
#include <RadioController.h>


#if defined(USE_BLACKBOX) || defined(USE_BLACKBOX_DEBUG)
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
#endif