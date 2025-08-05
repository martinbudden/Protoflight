#pragma once

#include <array>
#include <cstdint>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#endif

class FlightControllerMessageQueue {
public:
    struct queue_item_t {
        float roll;
        float pitch;
        float yaw;
    };
public:
#if defined(USE_FREERTOS)
    FlightControllerMessageQueue()
        : _queue(xQueueCreateStatic(QUEUE_LENGTH, sizeof(_queueItem), &_queueStorageArea[0], &_queueStatic))
    {}
    inline int32_t RECEIVE(queue_item_t& queueItem) const { return xQueueReceive(_queue, &queueItem, portMAX_DELAY); }
    inline void OVERWRITE(const queue_item_t& queueItem) const { xQueueOverwrite(_queue, &queueItem); }
#else
    FlightControllerMessageQueue() = default;
    inline int32_t RECEIVE(queue_item_t& queueItem) const { queueItem = {}; return 0; }
    inline void OVERWRITE(const queue_item_t& queueItem) const { (void)queueItem; }
#endif // USE_FREERTOS
private:
    mutable queue_item_t _queueItem {}; // this is just a dummy item whose value is not used
    enum { QUEUE_LENGTH = 1 };
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_queueItem)> _queueStorageArea {};
#if defined(USE_FREERTOS)
    StaticQueue_t _queueStatic {};
    QueueHandle_t _queue {};
#endif
};
