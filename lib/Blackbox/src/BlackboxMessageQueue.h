#pragma once

#include <BlackboxMessageQueueBase.h>

#include <array>
#include <xyz_type.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_USE_FREERTOS_SUBDIRECTORY)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#else
#include <FreeRTOS.h>
#include <queue.h>
#endif
#endif


class BlackboxMessageQueue : public BlackboxMessageQueueBase {
public:
    struct queue_item_t {
        uint32_t timeMicroSeconds;
        xyz_t gyroRPS;
        xyz_t gyroRPS_unfiltered;
        xyz_t acc;
    };
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    BlackboxMessageQueue()
        : _queue(xQueueCreateStatic(QUEUE_LENGTH, sizeof(_queueItem), &_queueStorageArea[0], &_queueStatic))
    {}
    virtual int32_t WAIT_IF_EMPTY(uint32_t& timeMicroSeconds) const override {
        const int32_t ret = xQueuePeek(_queue, &_queueItem, portMAX_DELAY);
        if (ret) {
            timeMicroSeconds = _queueItem.timeMicroSeconds;
        }
        return ret;
    }
    inline int32_t RECEIVE(queue_item_t& queueItem) const { return xQueueReceive(_queue, &queueItem, portMAX_DELAY); }
    inline void SEND(const queue_item_t& queueItem) const { xQueueSend(_queue, &queueItem, portMAX_DELAY); }
    inline bool SEND_IF_NOT_FULL(const queue_item_t& queueItem) const {
        if (uxQueueSpacesAvailable(_queue)) {
            xQueueSend(_queue, &queueItem, portMAX_DELAY);
            return true;
        }
        return false;
    }
#else
    BlackboxMessageQueue() = default;
    virtual int32_t WAIT_IF_EMPTY(uint32_t& timeMicroSeconds) const override { timeMicroSeconds = 0; return 0; }
    inline int32_t RECEIVE(queue_item_t& queueItem) const { queueItem = {}; return 0; }
    inline void SEND(const queue_item_t& queueItem) const { (void)queueItem; }
    inline bool SEND_IF_NOT_FULL(const queue_item_t& queueItem) const { (void)queueItem; return false; } // cppcheck-suppress knownConditionTrueFalse
#endif // USE_FREERTOS
private:
    mutable queue_item_t _queueItem {}; // this is just a dummy item whose value is not used
    enum { QUEUE_LENGTH = 8 };
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_queueItem)> _queueStorageArea {};
#if defined(FRAMEWORK_USE_FREERTOS)
    StaticQueue_t _queueStatic {};
    QueueHandle_t _queue {};
#endif
};
