#include "MSP_Serial.h"
#include "MSP_SerialPortBase.h"
#include <StreamBuf.h>


/*!
Called from MSP_Task::loop()
*/
void MSP_Serial::processInput()
{
    while (_mspSerialPort.isDataAvailable()) {
        const uint8_t inChar = _mspSerialPort.readByte();
        _mspStream.putChar(inChar, nullptr); // This will invoke MSP_Serial::sendFrame(), when a completed frame is received
    }
}

/*!
Called from  MSP_Stream::serialEncode() which is called from MSP_Stream::processReceivedCommand() which is called from MSP_Stream::putChar()
*/
size_t MSP_Serial::sendFrame(const uint8_t* hdr, size_t hdrLen, const uint8_t* data, size_t dataLen, const uint8_t* crc, size_t crcLen)
{
    const size_t totalFrameLength = hdrLen + dataLen + crcLen;

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

    while (sbuf.bytesRemaining() > 0) {
        const size_t available = _mspSerialPort.availableForWrite();
        const size_t writeLen = std::min(available, sbuf.bytesRemaining());
        _mspSerialPort.write(sbuf.ptr(), writeLen);
        sbuf.advance(writeLen);
        //!!delayMs(1);
    }

    return totalFrameLength;
}
