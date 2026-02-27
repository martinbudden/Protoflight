#include "Main.h"
#include "NonVolatileStorage.h"
#include "RX.h"

#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <receiver_atom_joystick.h>
#include <receiver_crsf.h>
#include <receiver_ibus.h>
#include <receiver_sbus.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <WiFi.h>
#endif

#include <cassert>


ReceiverBase& Main::createReceiver(NonVolatileStorage& nvs)
{
    (void)nvs;
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)

    // Set WiFi to station mode
    WiFi.mode(WIFI_STA);
    // Disconnect from Access Point if it was previously connected
    WiFi.disconnect();
    // get my MAC address
    std::array<uint8_t, ESP_NOW_ETH_ALEN> my_mac_address {};
    Network.macAddress(&my_mac_address[0]);
#if !defined(RECEIVER_CHANNEL)
    enum { RECEIVER_CHANNEL = 3 };
#endif
    static ReceiverAtomJoystick receiver(&my_mac_address[0], RECEIVER_CHANNEL);
    receiver.set_positive_half_throttle(true);
    const esp_err_t espErr = receiver.init();
    Serial.printf("\r\n\r\n**** ESP-NOW Ready:%X\r\n\r\n", espErr);
    assert(espErr == ESP_OK && "Unable to setup receiver.");
#if defined(M5_UNIFIED)
    // Holding BtnB down while switching on initiates binding.
    // The Atom has no BtnB, so it always broadcasts address for binding on startup.
    if (M5.getBoard() ==lgfx::board_M5AtomS3 || M5.BtnB.wasPressed()) {
        receiver.broadcast_my_eui();
    }
#else
    // no buttons defined, so always broadcast address for binding on startup
    receiver.broadcast_my_eui();
#endif // M5_UNIFIED
    return receiver;

#elif defined(USE_RECEIVER_SBUS)

    static SerialPort serialPort(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverSbus::BAUD_RATE, ReceiverSbus::DATA_BITS, ReceiverSbus::STOP_BITS, ReceiverSbus::PARITY);
    static ReceiverSbus receiver(serialPort);
    return receiver;

#elif defined(USE_RECEIVER_IBUS)

    static SerialPort serialPort(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverIbus::BAUD_RATE, ReceiverIbus::DATA_BITS, ReceiverIbus::STOP_BITS, ReceiverIbus::PARITY);
    static ReceiverIbus receiver(serialPort);
    return receiver;

#elif defined(USE_RECEIVER_CRSF)

    static SerialPort serialPort(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverCrsf::BAUD_RATE, ReceiverCrsf::DATA_BITS, ReceiverCrsf::STOP_BITS, ReceiverCrsf::PARITY);
    static ReceiverCrsf receiver(serialPort);
    return receiver;

#else

    const rx_config_t& rxConfig = nvs.load_rx_config();
    auto rxType = static_cast<RX::serial_type_e>(rxConfig.serial_rx_provider);

    uint32_t baudrate = ReceiverCrsf::BAUD_RATE;
    uint8_t dataBits = ReceiverCrsf::DATA_BITS;
    uint8_t stopBits = ReceiverCrsf::STOP_BITS;
    uint8_t parity = ReceiverCrsf::PARITY;

    switch (rxType) {
    case RX::SERIAL_SBUS:
        baudrate = ReceiverSbus::BAUD_RATE;
        dataBits = ReceiverSbus::DATA_BITS;
        stopBits = ReceiverSbus::STOP_BITS;
        parity   = ReceiverSbus::PARITY;
        break;
    case RX::SERIAL_IBUS:
        baudrate = ReceiverIbus::BAUD_RATE;
        dataBits = ReceiverIbus::DATA_BITS;
        stopBits = ReceiverIbus::STOP_BITS;
        parity   = ReceiverIbus::PARITY;
        break;
    default:
        break;
    }

    static SerialPort serialPort(SerialPort::RECEIVER_PINS, RECEIVER_UART_INDEX, baudrate, dataBits, stopBits, parity);

    // statically allocate the memory for the receiver
    // Note: difference between size of smallest receiver and largest receiver is only 80 bytes, so using the max size is not that wasteful.
    static constexpr size_t RECEIVER_SIZE = std::max({sizeof(ReceiverSbus), sizeof(ReceiverIbus), sizeof(ReceiverCrsf)});
    static std::array<uint8_t, RECEIVER_SIZE> receiverMemory;

    // The receiver will exist for the duration of the program and so never needs to be deleted, so it is OK to leave its pointer dangling.
    ReceiverBase* receiverPtr = nullptr;
    switch (rxType) {
    case RX::SERIAL_SBUS:
        receiverPtr = new(&receiverMemory[0]) ReceiverSbus(serialPort);
        break;
    case RX::SERIAL_IBUS:
        receiverPtr = new(&receiverMemory[0]) ReceiverIbus(serialPort);
        break;
    case RX::SERIAL_CRSF:
        receiverPtr = new(&receiverMemory[0]) ReceiverCrsf(serialPort);
        break;
    default:
        assert(false && "Receiver type not supported");
        break;
    }
    return *receiverPtr;

#endif
}
