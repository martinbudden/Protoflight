#include "OSD.h"
#include "OSD_Task.h"

#include <TimeMicroseconds.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <task.h>
#endif
#endif


/*!
loop() function for when not using FREERTOS
*/
void OSD_Task::loop()
{
    const timeUs32_t timeMicroseconds = timeUs();
    _tickCountDelta = timeMicroseconds - _timeMicrosecondsPrevious;

    if (_timeMicrosecondsDelta >= _taskIntervalMicroseconds) { // if _taskIntervalMicroseconds has passed, then run the update
        _timeMicrosecondsPrevious = timeMicroseconds;
        _osd.updateDisplay(timeMicroseconds, _timeMicrosecondsDelta);
    }
}

/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
[[noreturn]] void OSD_Task::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    const uint32_t taskIntervalTicks = _taskIntervalMicroseconds < 1000 ? 1 : pdMS_TO_TICKS(_taskIntervalMicroseconds / 1000);
    _previousWakeTimeTicks = xTaskGetTickCount();
    while (true) {
        // delay until the end of the next taskIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);

        // record tickCounts for instrumentation. Not sure if this is useful anymore
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const timeUs32_t timeMicroseconds = timeUs();
        _timeMicrosecondsDelta = timeMicroseconds - _timeMicrosecondsPrevious;
        _timeMicrosecondsPrevious = timeMicroseconds;
        if (_timeMicrosecondsDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
            _osd.updateDisplay(timeMicroseconds, _timeMicrosecondsDelta);
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for AHRS::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void OSD_Task::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<OSD_Task*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
