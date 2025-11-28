#pragma once

#include <MSP_SerialBase.h>
#include <MSP_Stream.h>

#include <array>

class MSP_SerialPortBase;


class MSP_Serial : public MSP_SerialBase {
public:
    explicit MSP_Serial(MSP_Stream& mspStream, MSP_SerialPortBase& mspSerialPort) : _mspStream(mspStream), _mspSerialPort(mspSerialPort) {}
    virtual size_t sendFrame(const uint8_t* hdr, size_t hdrLen, const uint8_t* data, size_t dataLen, const uint8_t* crc, size_t crcLen) override;
    virtual void processInput() override;
private:
    MSP_Stream& _mspStream;
    MSP_SerialPortBase& _mspSerialPort;
    std::array<uint8_t, 256> _buffer {};
};