#pragma once

#include "gps_message_data.h"
#include <array>
#include <cstdint>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <queue.h>
#endif
#endif

class GPS_MessageQueue {
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    GPS_MessageQueue() {
        _queue_handle = xQueueCreateStatic(QUEUE_LENGTH, sizeof(gps_message_data_t), &_queue_storage_area[0], &_queue_static);
    }
    // typical xQueueOverwrite overhead ~50 CPU cycles for 60-byte queue
    inline void SEND_GPS_DATA(const gps_message_data_t& gps_data) { xQueueOverwrite(_queue_handle, &gps_data); }
    // typical xQueuePeek overhead ~40 CPU cycles for 60-byte queue
    inline int32_t PEEK_GPS_DATA(gps_message_data_t& gps_data) const { return xQueuePeek(_queue_handle, &gps_data, portMAX_DELAY); }
    inline int32_t WAIT(gps_message_data_t& gps_data) { return xQueueReceive(_queue_handle, &gps_data, portMAX_DELAY); }
private:
    enum { QUEUE_LENGTH = 1 };
    QueueHandle_t _queue_handle;
    StaticQueue_t _queue_static {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(gps_message_data_t)> _queue_storage_area {};
#else
    GPS_MessageQueue() = default;
    inline void SEND_GPS_DATA(const gps_message_data_t& gps_message_data) { _gps_message_data = gps_message_data; }
    inline int32_t PEEK_GPS_DATA(gps_message_data_t& gps_message_data) const { gps_message_data = _gps_message_data; return 0; }
private:
    gps_message_data_t _gps_message_data {};
#endif // USE_FREERTOS
};
