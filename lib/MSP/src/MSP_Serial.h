#pragma once

#include <MSP_SerialBase.h>
#include <MSP_Stream.h>

#include <array>


class MSP_Serial : public MSP_SerialBase {
public:
    explicit MSP_Serial(MSP_Stream& mspStream) : _mspStream(mspStream) {}

    virtual size_t sendFrame(const uint8_t* hdr, size_t hdrLen, const uint8_t* data, size_t dataLen, const uint8_t* crc, size_t crcLen) override;

    virtual void processInput() override;

private:
    MSP_Stream& _mspStream;
#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_RPI_PICO)
#endif
#endif // FRAMEWORK
    std::array<uint8_t, 256> _buffer {};
};