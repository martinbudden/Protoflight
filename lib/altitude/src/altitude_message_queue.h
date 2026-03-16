#pragma once

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

struct altitude_data_t {
    float altitude_meters;
    float altitude_velocity_meters_per_second;
};

class AltitudeMessageQueue {
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    AltitudeMessageQueue() {
        _altitude_data_queue_handle = xQueueCreateStatic(QUEUE_LENGTH, sizeof(_altitude_data), &_altitude_data_queue_storage_area[0], &_altitude_data_queue_static);
    }
    // typical xQueueOverwrite overhead ~50 CPU cycles for 60-byte queue
    inline void SEND_ALTITUDE_DATA(const altitude_data_t& altitude_data) { xQueueOverwrite(_altitude_data_queue_handle, &altitude_data); }
    // typical xQueuePeek overhead ~40 CPU cycles for 60-byte queue
    inline int32_t PEEK_ALTITUDE_DATA(altitude_data_t& altitude_data) const { return xQueuePeek(_altitude_data_queue_handle, &altitude_data, portMAX_DELAY); }
#else
    AltitudeMessageQueue() = default;
    inline void SEND_ALTITUDE_DATA(const altitude_data_t& altitude_data) { _altitude_data = altitude_data; }
    inline int32_t PEEK_ALTITUDE_DATA(altitude_data_t& altitude_data) const { altitude_data = _altitude_data; return 0; }
#endif // USE_FREERTOS
private:
    altitude_data_t _altitude_data {};
#if defined(FRAMEWORK_USE_FREERTOS)
    enum { QUEUE_LENGTH = 1 };
    QueueHandle_t _synchronization_queue_handle;
    StaticQueue_t _synchronization_queue_static {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_altitude_data)> _synchronization_queue_storage_area {};
    QueueHandle_t _altitude_data_queue_handle;
    StaticQueue_t _altitude_data_queue_static {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_altitude_data)> _altitude_data_queue_storage_area {};
#endif
};
