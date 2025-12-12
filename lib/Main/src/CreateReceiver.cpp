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
    std::array<uint8_t, ESP_NOW_ETH_ALEN> myMacAddress {};
    Network.macAddress(&myMacAddress[0]);
#if !defined(RECEIVER_CHANNEL)
    enum { RECEIVER_CHANNEL = 3 };
#endif
    static ReceiverAtomJoyStick receiver(&myMacAddress[0], RECEIVER_CHANNEL);
    receiver.setPositiveHalfThrottle(true);
    const esp_err_t espErr = receiver.init();
    Serial.printf("\r\n\r\n**** ESP-NOW Ready:%X\r\n\r\n", espErr);
    assert(espErr == ESP_OK && "Unable to setup receiver.");
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

#elif defined(USE_RECEIVER_SBUS)

    static ReceiverSBUS receiver(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverSBUS::BAUD_RATE);
    return receiver;

#elif defined(USE_RECEIVER_IBUS)

    static ReceiverIBUS receiver(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverIBUS::BAUD_RATE);
    return receiver;

#elif defined(USE_RECEIVER_CRSF)

    static ReceiverCRSF receiver(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverCRSF::BAUD_RATE);
    return receiver;

#else

    // The receiver will exist for the duration of the program and so never needs to be deleted, so it is OK to leave its pointer dangling.
    ReceiverBase* receiverPtr = nullptr;
    const Cockpit::rx_config_t& rxConfig = nvs.loadRX_Config();
    auto rxType = static_cast<Cockpit::serial_rx_type>(rxConfig.serial_rx_type);
    switch (rxType) {
    case Cockpit::SERIAL_RX_SBUS:
        receiverPtr = new ReceiverSBUS(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverSBUS::BAUD_RATE);
        break;
    case Cockpit::SERIAL_RX_IBUS:
        receiverPtr = new ReceiverIBUS(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverIBUS::BAUD_RATE);
        break;
    case Cockpit::SERIAL_RX_CRSF:
        receiverPtr = new ReceiverCRSF(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverCRSF::BAUD_RATE);
        break;
    default:
        assert(false && "Receiver type not supported");
        break;
    }
    assert((receiverPtr != nullptr) && "Receiver could not be created");
    return *receiverPtr;

#endif
}
