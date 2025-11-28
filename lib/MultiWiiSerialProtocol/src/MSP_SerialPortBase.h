#pragma once

#include <cstddef>
#include <cstdint>


class MSP_SerialPortBase {
public:
    virtual bool isDataAvailable() const;
    virtual uint8_t readByte();
    virtual size_t availableForWrite() const;
    virtual size_t write(uint8_t* buf, size_t len);
};
