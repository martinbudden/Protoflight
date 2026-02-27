#pragma once

#include "RX.h"

#include <time_microseconds.h>

#include <array>


class RX_Base {
public:
    virtual float rcReadRawData(uint8_t chan) = 0; // used by receiver driver to return channel data
    virtual uint8_t rcFrameStatus() = 0;
    virtual bool rcProcessFrame() = 0;
    time_us32_t rcGetFrameTime();  // used to retrieve the timestamp in microseconds for the last channel data frame
private:
    time_us32_t          _lastRcFrameTimeUs;
    RX::provider_e      _rxProvider;
    RX::serial_type_e   _serialProvider;
    uint8_t             _channelCount; // number of RC channels as reported by current input driver
};
