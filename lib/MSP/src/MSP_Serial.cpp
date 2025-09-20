#include "MSP_Serial.h"

#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#endif


/*!
Called from MSP_Task::loop()
*/
void MSP_Serial::processInput()
{
#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    while (Serial.available() > 0) {
        const uint8_t inChar = Serial.read();
        _mspStream.putChar(inChar, nullptr); // This will invoke MSP_Serial::sendFrame(), when a completed frame is received
    }
#endif
}

/*!
Called from  MSP_Stream::serialEncode() which is called from MSP_Stream::processReceivedCommand() which is called from MSP_Stream::putChar()
*/
int MSP_Serial::sendFrame(const uint8_t* hdr, int hdrLen, const uint8_t* data, int dataLen, const uint8_t* crc, int crcLen)
{
    const int totalFrameLength = hdrLen + dataLen + crcLen;

    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    //if (!isSerialTransmitBufferEmpty(_serialPort) && ((int)serialTxBytesFree(_serialPort) < totalFrameLength)) {
    //    return 0;
    //}
    // buffer size is 64 bytes on arduino
    // buffer empty if Serial.availableForWrite() >= SERIAL_TX_BUFFER_SIZE - 1
    // if (totalFrameLength <= Serial.availableForWrite())

    StreamBuf sbuf(&_buffer[0], sizeof(_buffer));

    // copy the frame into a StreamBuf
    sbuf.writeData(hdr, hdrLen);
    sbuf.writeData(data, dataLen);
    sbuf.writeData(crc, crcLen);
    sbuf.switchToReader();

#if defined(FRAMEWORK_RPI_PICO)
    (void)sbuf;
#elif defined(FRAMEWORK_ESPIDF)
    (void)sbuf;
#elif defined(FRAMEWORK_STM32_CUBE)
    (void)sbuf;
#elif defined(FRAMEWORK_TEST)
    (void)sbuf;
#else // defaults to FRAMEWORK_ARDUINO
    while (sbuf.bytesRemaining() > 0) {
        const size_t available = Serial.availableForWrite();
        const size_t writeLen = std::min(available, sbuf.bytesRemaining());
        Serial.write(sbuf.ptr(), writeLen);
        sbuf.advance(writeLen);
        delay(1);
    }
#endif

    return totalFrameLength;
}
