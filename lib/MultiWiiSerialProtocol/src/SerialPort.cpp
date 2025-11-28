#include "SerialPort.h"

#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#endif

SerialPort::SerialPort(const uart_pins_t& pins, uint8_t uartIndex, uint32_t baudrate, uint8_t dataBits, uint8_t stopBits, uint8_t parity) : // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    _pins({
        .rx = { .port = pins.rx.port, .pin = static_cast<int8_t>(pins.rx.pin < 0 ? -pins.rx.pin : pins.rx.pin), .inverted = pins.rx.pin < 0 },
        .tx = { .port = pins.tx.port, .pin = static_cast<int8_t>(pins.tx.pin < 0 ? -pins.tx.pin : pins.tx.pin), .inverted = pins.tx.pin < 0 }
    }),
    _uartIndex(uartIndex),
    _dataBits(dataBits),
    _stopBits(stopBits),
    _parity(parity),
    _baudrate(baudrate)
#if defined(FRAMEWORK_ARDUINO_ESP32)
    ,_uart(uartIndex)
#endif
{
}

SerialPort::SerialPort(const stm32_rx_pins_t& pins, uint8_t uartIndex, uint32_t baudrate, uint8_t dataBits, uint8_t stopBits, uint8_t parity) :
    SerialPort(uart_pins_t{{pins.tx.port,pins.tx.pin,false},{pins.rx.port,pins.rx.pin,false}}, uartIndex, baudrate, dataBits, stopBits, parity)
{
}

SerialPort::SerialPort(const rx_pins_t& pins, uint8_t uartIndex, uint32_t baudrate, uint8_t dataBits, uint8_t stopBits, uint8_t parity) :
    SerialPort(uart_pins_t{{0,pins.tx,false},{0,pins.rx,false}}, uartIndex, baudrate, dataBits, stopBits, parity)
{
}

bool SerialPort::isDataAvailable()
{
#if defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    return uart_is_readable(_uart);
#elif defined(FRAMEWORK_ESPIDF)
    return false;
#elif defined(FRAMEWORK_STM32_CUBE)
    return (__HAL_UART_GET_FLAG(&_uart, UART_FLAG_RXNE)) ? true : false;
#elif defined(FRAMEWORK_TEST)
    return false;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    return _uart.available() > 0;
#else
    return Serial.available() > 0;
#endif
#endif
}

uint8_t SerialPort::readByte()
{
#if defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    return uart_getc(_uart);
#elif defined(FRAMEWORK_ESPIDF)
    return 0;
#elif defined(FRAMEWORK_STM32_CUBE)
#if false
    uint8_t data; // Buffer to store received data
    HAL_UART_Receive(&_uart, &data, sizeof(data), HAL_MAX_DELAY);
#else
    // read from uart
#if defined(FRAMEWORK_STM32_CUBE_F4)
    const uint8_t data = static_cast<uint8_t>(_uart.Instance->DR & 0xFF);
#else
    const uint8_t data = 0;
#endif
#endif
    return data;
#elif defined(FRAMEWORK_TEST)
    return 0;
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    return static_cast<uint8_t>(_uart.read());
#else
    return Serial.read();
#endif
#endif
}

size_t SerialPort::availableForWrite() const
{
#if defined(FRAMEWORK_RPI_PICO)
    return 0;
#elif defined(FRAMEWORK_ESPIDF)
    return 0;
#elif defined(FRAMEWORK_STM32_CUBE)
    return 0;
#elif defined(FRAMEWORK_TEST)
    return 0;
#else // defaults to FRAMEWORK_ARDUINO
    return Serial.availableForWrite();
#endif
}

size_t SerialPort::write(const uint8_t* buf, size_t len)
{
#if defined(FRAMEWORK_RPI_PICO)
    (void)buf;
    (void)len;
    return 0;
#elif defined(FRAMEWORK_ESPIDF)
    (void)buf;
    (void)len;
    return 0;
#elif defined(FRAMEWORK_STM32_CUBE)
    (void)buf;
    (void)len;
    return 0;
#elif defined(FRAMEWORK_TEST)
    (void)buf;
    (void)len;
    return 0;
#else // defaults to FRAMEWORK_ARDUINO
    return Serial.write(buf, len);
#endif
}
