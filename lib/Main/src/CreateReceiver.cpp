#include "Main.h"

#include <ReceiverAtomJoyStick.h>
#include <ReceiverCRSF.h>
#include <ReceiverIBUS.h>
#include <ReceiverNull.h>
#include <ReceiverSBUS.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <WiFi.h>
#endif


ReceiverBase& Main::createReceiver()
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
    receiver.setPositiveHalfThrottle(true);
    const esp_err_t espErr = receiver.init();
    Serial.printf("\r\n\r\n**** ESP-NOW Ready:%X\r\n\r\n", espErr);
    assert(espErr == ESP_OK && "Unable to setup receiver.");
#elif defined(USE_RECEIVER_CRSF)
    static ReceiverCRSF receiver(ReceiverSerial::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverCRSF::BAUD_RATE);
#elif defined(USE_RECEIVER_IBUS)
    static ReceiverIBUS receiver(ReceiverSerial::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverIBUS::BAUD_RATE);
#elif defined(USE_RECEIVER_SBUS)
    static ReceiverSBUS receiver(ReceiverSerial::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverSBUS::BAUD_RATE);
#else
    static ReceiverNull receiver;
#endif

    return receiver;
}
