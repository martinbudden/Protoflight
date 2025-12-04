#include "MSP_SerialPort.h"
#include "SerialPort.h"


bool MSP_SerialPort::isDataAvailable() const
{
    return _serialPort.isDataAvailable();
}

uint8_t MSP_SerialPort::readByte()
{
    return _serialPort.readByte();
}

size_t MSP_SerialPort::availableForWrite() const
{
    return _serialPort.availableForWrite();
}

size_t MSP_SerialPort::write(const uint8_t* buf, size_t len)
{
    return _serialPort.write(buf, len);
}

