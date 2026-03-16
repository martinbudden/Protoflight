#include "msp_serial_port.h"
#include <serial_port.h>


bool MSP_SerialPort::is_data_available() const
{
    return _serial_port.is_data_available();
}

uint8_t MSP_SerialPort::read_byte()
{
    return _serial_port.read_byte();
}

size_t MSP_SerialPort::available_for_write() const
{
    return _serial_port.available_for_write();
}

size_t MSP_SerialPort::write(const uint8_t* buf, size_t len)
{
    return _serial_port.write(buf, len);
}

