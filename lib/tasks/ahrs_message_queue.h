#pragma once

#include "ahrs_data.h"

#include <ahrs.h>
#include <array>
#include <message_queue_base.h>

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


class AhrsMessageQueue : public MessageQueueBase {
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    AhrsMessageQueue() {
        _synchronization_queue_handle = xQueueCreateStatic(QUEUE_LENGTH, sizeof(_ahrs_data), &_synchronization_queue_storage_area[0], &_synchronization_queue_static);
        _ahrs_data_queue_handle = xQueueCreateStatic(QUEUE_LENGTH, sizeof(_ahrs_data), &_ahrs_data_queue_storage_area[0], &_ahrs_data_queue_static);
    }
    // Blackbox Task calls WAIT and then `update` when wait completes
    virtual int32_t WAIT(uint32_t& time_microseconds) override {
        const int32_t ret = xQueueReceive(_synchronization_queue_handle, &_ahrs_data, portMAX_DELAY);
        if (ret) {
            time_microseconds = _ahrs_data.time_microseconds;
        }
        return ret;
    }
    // typical xQueueOverwrite overhead ~50 CPU cycles for 60-byte queue
    inline void SIGNAL(const ahrs_data_t& ahrs_data) { xQueueOverwrite(_synchronization_queue_handle, &ahrs_data); }
    inline void SEND_AHRS_DATA(const ahrs_data_t& ahrs_data) { xQueueOverwrite(_ahrs_data_queue_handle, &ahrs_data); }
    // typical xQueuePeek overhead ~40 CPU cycles for 60-byte queue
    inline int32_t PEEK_AHRS_DATA(ahrs_data_t& ahrs_data) const { return xQueuePeek(_ahrs_data_queue_handle, &ahrs_data, portMAX_DELAY); }
#else
    AhrsMessageQueue() = default;
    virtual int32_t WAIT(uint32_t& time_microseconds) override { time_microseconds = 0; return 0; }
    inline void SIGNAL(const ahrs_data_t& ahrs_data) { _ahrs_data = ahrs_data; }
    inline void SEND_AHRS_DATA(const ahrs_data_t& ahrs_data) { _ahrs_data = ahrs_data; }
    inline int32_t PEEK_AHRS_DATA(ahrs_data_t& ahrs_data) const { ahrs_data = _ahrs_data; return 0; }
#endif // USE_FREERTOS
    const ahrs_data_t& get_received_ahrs_data() const { return _ahrs_data; } //!< May only be called within task after WAIT has completed
private:
    ahrs_data_t _ahrs_data {};
#if defined(FRAMEWORK_USE_FREERTOS)
    enum { QUEUE_LENGTH = 1 };
    QueueHandle_t _synchronization_queue_handle;
    StaticQueue_t _synchronization_queue_static {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_ahrs_data)> _synchronization_queue_storage_area {};
    QueueHandle_t _ahrs_data_queue_handle;
    StaticQueue_t _ahrs_data_queue_static {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_ahrs_data)> _ahrs_data_queue_storage_area {};
#endif
};
