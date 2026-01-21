#include "Main.h"
#include "NonVolatileStorage.h"
#include "RX.h"

#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <ReceiverAtomJoyStick.h>
#include <ReceiverCRSF.h>
#include <ReceiverIBUS.h>
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

    static SerialPort serialPort(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverSBUS::BAUD_RATE, ReceiverSBUS::DATA_BITS, ReceiverSBUS::STOP_BITS, ReceiverSBUS::PARITY);
    static ReceiverSBUS receiver(serialPort);
    return receiver;

#elif defined(USE_RECEIVER_IBUS)

    static SerialPort serialPort(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverIBUS::BAUD_RATE, ReceiverIBUS::DATA_BITS, ReceiverIBUS::STOP_BITS, ReceiverIBUS::PARITY);
    static ReceiverIBUS receiver(serialPort);
    return receiver;

#elif defined(USE_RECEIVER_CRSF)

    static SerialPort serialPort(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverCRSF::BAUD_RATE, ReceiverCRSF::DATA_BITS, ReceiverCRSF::STOP_BITS, ReceiverCRSF::PARITY);
    static ReceiverCRSF receiver(serialPort);
    return receiver;

#else

    const RX::config_t& rxConfig = nvs.loadRX_Config();
    auto rxType = static_cast<RX::serial_type_e>(rxConfig.serial_rx_provider);

    uint32_t baudrate = ReceiverCRSF::BAUD_RATE;
    uint8_t dataBits = ReceiverCRSF::DATA_BITS;
    uint8_t stopBits = ReceiverCRSF::STOP_BITS;
    uint8_t parity = ReceiverCRSF::PARITY;

    switch (rxType) {
    case RX::SERIAL_SBUS:
        baudrate = ReceiverSBUS::BAUD_RATE;
        dataBits = ReceiverSBUS::DATA_BITS;
        stopBits = ReceiverSBUS::STOP_BITS;
        parity   = ReceiverSBUS::PARITY;
        break;
    case RX::SERIAL_IBUS:
        baudrate = ReceiverIBUS::BAUD_RATE;
        dataBits = ReceiverIBUS::DATA_BITS;
        stopBits = ReceiverIBUS::STOP_BITS;
        parity   = ReceiverIBUS::PARITY;
        break;
    default:
        break;
    }

    static SerialPort serialPort(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, baudrate, dataBits, stopBits, parity);

    // statically allocate the memory for the receiver
    // Note: difference between size of smallest receiver and largest receiver is only 80 bytes, so using the max size is not that wasteful.
    static constexpr size_t RECEIVER_SIZE = std::max({sizeof(ReceiverSBUS), sizeof(ReceiverIBUS), sizeof(ReceiverCRSF)});
    static std::array<uint8_t, RECEIVER_SIZE> receiverMemory;

    // The receiver will exist for the duration of the program and so never needs to be deleted, so it is OK to leave its pointer dangling.
    ReceiverBase* receiverPtr = nullptr;
    switch (rxType) {
    case RX::SERIAL_SBUS:
        receiverPtr = new(&receiverMemory[0]) ReceiverSBUS(serialPort);
        break;
    case RX::SERIAL_IBUS:
        receiverPtr = new(&receiverMemory[0]) ReceiverIBUS(serialPort);
        break;
    case RX::SERIAL_CRSF:
        receiverPtr = new(&receiverMemory[0]) ReceiverCRSF(serialPort);
        break;
    default:
        assert(false && "Receiver type not supported");
        break;
    }
    return *receiverPtr;

#endif
}
