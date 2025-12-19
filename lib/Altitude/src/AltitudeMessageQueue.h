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
    float altitudeMeters;
    float altitudeVelocityMetersPerSecond;
};

class AltitudeMessageQueue {
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    AltitudeMessageQueue() {
        _altitudeDataQueueHandle = xQueueCreateStatic(QUEUE_LENGTH, sizeof(_altitudeData), &_altitudeDataQueueStorageArea[0], &_altitudeDataQueueStatic);
    }
    // typical xQueueOverwrite overhead ~50 CPU cycles for 60-byte queue
    inline void SEND_ALTITUDE_DATA(const altitude_data_t& altitudeData) { xQueueOverwrite(_altitudeDataQueueHandle, &altitudeData); }
    // typical xQueuePeek overhead ~40 CPU cycles for 60-byte queue
    inline int32_t PEEK_ALTITUDE_DATA(altitude_data_t& altitudeData) const { return xQueuePeek(_altitudeDataQueueHandle, &altitudeData, portMAX_DELAY); }
#else
    AltitudeMessageQueue() = default;
    inline void SEND_ALTITUDE_DATA(const altitude_data_t& altitudeData) { _altitudeData = altitudeData; }
    inline int32_t PEEK_ALTITUDE_DATA(altitude_data_t& altitudeData) const { altitudeData = _altitudeData; return 0; }
#endif // USE_FREERTOS
    const altitude_data_t& getReceivedAHRS_Data() const { return _altitudeData; } //!< May only be called within task after WAIT has completed
private:
    altitude_data_t _altitudeData {};
#if defined(FRAMEWORK_USE_FREERTOS)
    enum { QUEUE_LENGTH = 1 };
    QueueHandle_t _synchronizationQueueHandle;
    StaticQueue_t _synchronizationQueueStatic {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_altitudeData)> _synchronizationQueueStorageArea {};
    QueueHandle_t _altitudeDataQueueHandle;
    StaticQueue_t _altitudeDataQueueStatic {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_altitudeData)> _altitudeDataQueueStorageArea {};
#endif
};
