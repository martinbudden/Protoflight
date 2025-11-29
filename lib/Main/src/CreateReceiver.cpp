#include "Cockpit.h"
#include "Main.h"
#include "NonVolatileStorage.h"

#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <ReceiverAtomJoyStick.h>
#include <ReceiverCRSF.h>
#include <ReceiverIBUS.h>
#include <ReceiverNull.h>
#include <ReceiverSBUS.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <WiFi.h>
#endif


ReceiverBase& Main::createReceiver(NonVolatileStorage& nvs)
{
    (void)nvs;
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

#elif defined(USE_RECEIVER_SBUS)

    static ReceiverSBUS receiver(ReceiverSerial::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverSBUS::BAUD_RATE);

#elif defined(USE_RECEIVER_IBUS)

    static ReceiverIBUS receiver(ReceiverSerial::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverIBUS::BAUD_RATE);

#elif defined(USE_RECEIVER_CRSF)

    static ReceiverCRSF receiver(ReceiverSerial::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverCRSF::BAUD_RATE);

#else

    static ReceiverNull receiver;
    const Cockpit::rx_config_t& rxConfig = nvs.loadRX_Config();
    Cockpit::serial_rx_type rxType = static_cast<Cockpit::serial_rx_type>(rxConfig.serial_rx_type);
    switch (rxType) {
    case Cockpit::SERIAL_RX_SBUS:
        break;
    case Cockpit::SERIAL_RX_IBUS:
        break;
    case Cockpit::SERIAL_RX_CRSF:
        break;
    default:
        break;
    }

#endif

#if defined(M5_UNIFIED)
    // Holding BtnB down while switching on initiates binding.
    // The Atom has no BtnB, so it always broadcasts address for binding on startup.
    if (M5.getBoard() ==lgfx::board_M5AtomS3 || M5.BtnB.wasPressed()) {
        receiver.broadcastMyEUI();
    }
#else
    // no buttons defined, so always broadcast address for binding on startup
    receiver.broadcastMyEUI();
#endif // M5_UNIFIED

    return receiver;
}
