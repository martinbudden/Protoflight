#pragma once

#include <msp_serial_port_base.h>

class SerialPort;


class MSP_SerialPort : public MspSerialPortBase {
public:
    MSP_SerialPort(SerialPort& serialPort) : _serialPort(serialPort) {}
private:
    // Receiver is not copyable or moveable
    MSP_SerialPort(const MSP_SerialPort&) = delete;
    MSP_SerialPort& operator=(const MSP_SerialPort&) = delete;
    MSP_SerialPort(MSP_SerialPort&&) = delete;
    MSP_SerialPort& operator=(MSP_SerialPort&&) = delete;
public:
    bool is_data_available() const override;
    uint8_t read_byte() override;
    size_t available_for_write() const override;
    size_t write(const uint8_t* buf, size_t len) override;
private:
    SerialPort& _serialPort;
};
