#include <BackchannelFlightController.h>
#include <BackchannelTask.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <BackchannelTransceiverESPNOW.h>
#endif
#include <FlightController.h>
#include <NonVolatileStorage.h>
#include <ReceiverAtomJoyStick.h>
#include <ReceiverBase.h>
#include <ReceiverNull.h>
#include <ReceiverSBUS.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <WiFi.h>
#endif
#include "Main.h"


RadioController& Main::createRadioController(FlightController& flightController, const NonVolatileStorage& nonVolatileStorage, uint8_t currentRateProfile)
{
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
    // Set WiFi to station mode
    WiFi.mode(WIFI_STA);
    // Disconnect from Access Point if it was previously connected
    WiFi.disconnect();
    // get my MAC address
    uint8_t myMacAddress[ESP_NOW_ETH_ALEN];
    WiFi.macAddress(&myMacAddress[0]);
#if !defined(RECEIVER_CHANNEL)
    enum { RECEIVER_CHANNEL = 3 };
#endif
    static ReceiverAtomJoyStick receiver(&myMacAddress[0], RECEIVER_CHANNEL);
    const esp_err_t espErr = receiver.init();
    Serial.printf("\r\n\r\n**** ESP-NOW Ready:%X\r\n\r\n", espErr);
    assert(espErr == ESP_OK && "Unable to setup receiver.");
#elif defined(USE_RECEIVER_SBUS)
    static ReceiverSBUS receiver(ReceiverSerial::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverSBUS::BAUD_RATE); // NOLINT(misc-const-correctness)
#else
    static ReceiverNull receiver;
#endif

    static RadioController radioController(receiver, flightController, nonVolatileStorage.loadRadioControllerRates(currentRateProfile));

    return radioController;
}

#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
BackchannelBase& Main::createBackchannel(FlightController& flightController, AHRS& ahrs, ReceiverBase& receiver, const TaskBase* mainTask, NonVolatileStorage& nonVolatileStorage)
{
    // statically allocate an MSP object
    // static MSP_ProtoFlight mspProtoFlightBackchannel(features, ahrs, flightController, radioController, receiver);
    // Statically allocate the backchannel.
    constexpr uint8_t backchannelMacAddress[ESP_NOW_ETH_ALEN] BACKCHANNEL_MAC_ADDRESS;
    auto& receiverAtomJoyStick = static_cast<ReceiverAtomJoyStick&>(receiver); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    static BackchannelTransceiverESPNOW backchannelTransceiverESPNOW(receiverAtomJoyStick.getESPNOW_Transceiver(), &backchannelMacAddress[0]);
    static BackchannelFlightController backchannel(
        backchannelTransceiverESPNOW,
        &backchannelMacAddress[0],
        //&myMacAddress[0],
        &receiver.getMyEUI().octets[0],
        flightController,
        ahrs,
        receiver,
        mainTask,
        nonVolatileStorage
    );

    return backchannel;
}
#endif
