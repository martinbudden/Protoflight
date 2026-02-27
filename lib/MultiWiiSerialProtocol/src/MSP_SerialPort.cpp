#include "MSP_SerialPort.h"
#include <serial_port.h>


bool MSP_SerialPort::is_data_available() const
{
    return _serialPort.is_data_available();
}

uint8_t MSP_SerialPort::read_byte()
{
    return _serialPort.read_byte();
}

size_t MSP_SerialPort::available_for_write() const
{
    return _serialPort.available_for_write();
}

size_t MSP_SerialPort::write(const uint8_t* buf, size_t len)
{
    return _serialPort.write(buf, len);
}

