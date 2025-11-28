#pragma once

#include <cstddef>
#include <cstdint>


#if defined(FRAMEWORK_RPI_PICO)
#include <hardware/uart.h>
#include <pico/mutex.h>
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)

#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_uart.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_gpio.h>
#include <stm32f3xx_hal_uart.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_uart.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_gpio.h>
#include <stm32f7xx_hal_uart.h>
#endif
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <HardwareSerial.h>
#endif
#endif

/*!
Negative pin means it is inverted.
*/
class SerialPort {
public:
    enum uart_index_e : uint8_t { UART_INDEX_0, UART_INDEX_1, UART_INDEX_2, UART_INDEX_3, UART_INDEX_4, UART_INDEX_5, UART_INDEX_6, UART_INDEX_7 };
    enum { PARITY_NONE, PARITY_EVEN, PARITY_ODD };
private:
    struct port_pin_t {
        int8_t port;
        int8_t pin;
    };
public:
    struct rx_pins_t {
        int8_t rx;
        int8_t tx;
    };
    struct stm32_rx_pins_t {
        port_pin_t rx;
        port_pin_t tx;
    };
private:
    struct uart_pin_t {
        int8_t port;
        int8_t pin;
        bool inverted;
    };
    struct uart_pins_t {
        uart_pin_t rx;
        uart_pin_t tx;
    };
public:
    SerialPort(const stm32_rx_pins_t& pins, uint8_t uartIndex, uint32_t baudrate, uint8_t dataBits, uint8_t stopBits, uint8_t parity);
    SerialPort(const rx_pins_t& pins, uint8_t uartIndex, uint32_t baudrate, uint8_t dataBits, uint8_t stopBits, uint8_t parity);
private:
    SerialPort(const uart_pins_t& pins, uint8_t uartIndex, uint32_t baudrate, uint8_t dataBits, uint8_t stopBits, uint8_t parity);
    // Receiver is not copyable or moveable
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&&) = delete;
    SerialPort& operator=(SerialPort&&) = delete;
public:
    bool isDataAvailable(); // not const, since ESP32 variant not const
    uint8_t readByte();
    size_t availableForWrite() const;
    size_t write(const uint8_t* buf, size_t len);
private:
    const uart_pins_t _pins {};
    const uint8_t _uartIndex;
    const uint8_t _dataBits;
    const uint8_t _stopBits;
    const uint8_t _parity;
    const uint32_t _baudrate;
#if defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    uart_inst_t* _uart {};
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    UART_HandleTypeDef _uart {};
    uint8_t _rxByte {};
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_ESP32)
    HardwareSerial _uart;
#endif
#endif
};
