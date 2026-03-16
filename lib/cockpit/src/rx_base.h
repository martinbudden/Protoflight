#pragma once

#include "rx.h"

#include <time_microseconds.h>

#include <array>


class RX_Base {
public:
    virtual float rc_read_raw_data(uint8_t chan) = 0; // used by receiver driver to return channel data
    virtual uint8_t rc_frame_status() = 0;
    virtual bool rc_process_frame() = 0;
    time_us32_t rc_get_frame_time();  // used to retrieve the timestamp in microseconds for the last channel data frame
private:
    time_us32_t          _last_rc_frame_time_is;
    RX::provider_e      _rx_provider;
    RX::serial_type_e   _serial_provider;
    uint8_t             _channel_count; // number of RC channels as reported by current input driver
};
