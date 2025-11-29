#pragma once

#include <MSP_SerialPortBase.h>

class SerialPort;


class MSP_SerialPort : public MSP_SerialPortBase {
public:
    MSP_SerialPort(SerialPort& serialPort) : _serialPort(serialPort) {}
private:
    // Receiver is not copyable or moveable
    MSP_SerialPort(const MSP_SerialPort&) = delete;
    MSP_SerialPort& operator=(const MSP_SerialPort&) = delete;
    MSP_SerialPort(MSP_SerialPort&&) = delete;
    MSP_SerialPort& operator=(MSP_SerialPort&&) = delete;
public:
    bool isDataAvailable() const override;
    uint8_t readByte() override;
    size_t availableForWrite() const override;
    size_t write(uint8_t* buf, size_t len) override;
private:
    SerialPort& _serialPort;
};
