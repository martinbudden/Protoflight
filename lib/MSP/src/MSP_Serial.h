#pragma once

#include "MSP_SerialBase.h"
#include "MSP_Stream.h"

#include <array>


class MSP_Serial : public MSP_SerialBase {
public:
    explicit MSP_Serial(MSP_Stream& mspStream) : _mspStream(mspStream) {}

    virtual int sendFrame(const uint8_t* hdr, int hdrLen, const uint8_t* data, int dataLen, const uint8_t* crc, int crcLen) override;

    virtual void processInput() override;

private:
    MSP_Stream& _mspStream;
    std::array<uint8_t, 256> _buffer {};
};