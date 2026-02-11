#include "MSP_SerialPort.h"
#include "SerialPort.h"


bool MSP_SerialPort::isDataAvailable() const
{
    return _serialPort.is_data_available();
}

uint8_t MSP_SerialPort::readByte()
{
    return _serialPort.read_byte();
}

size_t MSP_SerialPort::availableForWrite() const
{
    return _serialPort.available_for_write();
}

size_t MSP_SerialPort::write(const uint8_t* buf, size_t len)
{
    return _serialPort.write(buf, len);
}

