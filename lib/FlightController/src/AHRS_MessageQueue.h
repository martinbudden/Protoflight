#pragma once

#include <AHRS.h>
#include <MessageQueueBase.h>

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


class AHRS_MessageQueue : public MessageQueueBase {
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    AHRS_MessageQueue() :
        _queueHandle(xQueueCreateStatic(QUEUE_LENGTH, sizeof(_ahrsData), &_queueStorageArea[0], &_queueStatic)),
        _copyQueueHandle(xQueueCreateStatic(QUEUE_LENGTH, sizeof(_ahrsData), &_copyQueueStorageArea[0], &_copyQueueStatic))
    {}
    // Blackbox Task calls WAIT and then `update` when wait completes
    virtual int32_t WAIT(uint32_t& timeMicroseconds) const override {
        const int32_t ret = xQueueReceive(_queueHandle, &_ahrsData, portMAX_DELAY);
        if (ret) {
            timeMicroseconds = _ahrsData.timeMicroseconds;
        }
        return ret;
    }
    inline void SEND(const AHRS::ahrs_data_t& ahrsData) const { xQueueOverwrite(_queueHandle, &ahrsData); xQueueOverwrite(_copyQueueHandle, &ahrsData);}
    inline int32_t PEEK_COPY(AHRS::ahrs_data_t& ahrsData) const { return xQueuePeek(_copyQueueHandle, &ahrsData, portMAX_DELAY); }
#else
    AHRS_MessageQueue() = default;
    virtual int32_t WAIT(uint32_t& timeMicroseconds) const override { timeMicroseconds = 0; return 0; }
    inline void SEND(const AHRS::ahrs_data_t& ahrsData) const { _ahrsData = ahrsData; }
    inline int32_t PEEK_COPY(AHRS::ahrs_data_t& ahrsData) const { ahrsData = _ahrsData; return 0; }
#endif // USE_FREERTOS
    const AHRS::ahrs_data_t& getAHRS_Data() const { return _ahrsData; } //!< May only be called within task after WAIT has completed
private:
    mutable AHRS::ahrs_data_t _ahrsData {};
#if defined(FRAMEWORK_USE_FREERTOS)
    enum { QUEUE_LENGTH = 1 };
    QueueHandle_t _queueHandle {};
    StaticQueue_t _queueStatic {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_ahrsData)> _queueStorageArea {};
    QueueHandle_t _copyQueueHandle {};
    StaticQueue_t _copyQueueStatic {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_ahrsData)> _copyQueueStorageArea {};
#endif
};
