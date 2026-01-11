#pragma once

#include "GPS_MessageData.h"
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
        _queueHandle = xQueueCreateStatic(QUEUE_LENGTH, sizeof(gps_message_data_t), &_queueStorageArea[0], &_queueStatic);
    }
    // typical xQueueOverwrite overhead ~50 CPU cycles for 60-byte queue
    inline void SEND_GPS_DATA(const gps_message_data_t& gpsData) { xQueueOverwrite(_queueHandle, &gpsData); }
    // typical xQueuePeek overhead ~40 CPU cycles for 60-byte queue
    inline int32_t PEEK_GPS_DATA(gps_message_data_t& gpsData) const { return xQueuePeek(_queueHandle, &gpsData, portMAX_DELAY); }
    inline int32_t WAIT(gps_message_data_t& gpsData) { return xQueueReceive(_queueHandle, &gpsData, portMAX_DELAY); }
private:
    enum { QUEUE_LENGTH = 1 };
    QueueHandle_t _queueHandle;
    StaticQueue_t _queueStatic {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(gps_message_data_t)> _queueStorageArea {};
#else
    GPS_MessageQueue() = default;
    inline void SEND_GPS_DATA(const gps_message_data_t& gpsData) { _gpsData = gpsData; }
    inline int32_t PEEK_GPS_DATA(gps_message_data_t& gpsData) const { gpsData = _gpsData; return 0; }
private:
    gps_message_data_t _gpsData {};
#endif // USE_FREERTOS
};
