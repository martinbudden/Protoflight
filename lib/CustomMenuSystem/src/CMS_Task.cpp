#include "CMS.h"
#include "CMS_Task.h"

#include <time_microseconds.h>

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
void CMS_Task::loop()
{
    const time_us32_t time_microseconds = time_us();
    _tick_count_delta = time_microseconds - _time_microseconds_previous;

    if (_time_microseconds_delta >= _task_interval_microseconds) { // if _task_interval_microseconds has passed, then run the update
        _time_microseconds_previous = time_microseconds;
        _cms.updateCMS(_parameter_group, time_microseconds, _time_microseconds_delta);
    }
}

/*!
Task function for the CMS. Sets up and runs the task loop() function.
*/
[[noreturn]] void CMS_Task::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    const uint32_t taskIntervalTicks = _task_interval_microseconds < 1000 ? 1 : pdMS_TO_TICKS(_task_interval_microseconds / 1000);
    _previous_wake_time_ticks = xTaskGetTickCount();
    while (true) {
        // delay until the end of the next taskIntervalTicks
#define FRAMEWORK_FREERTOS_VERSION (tskKERNEL_VERSION_MAJOR * 10000 + tskKERNEL_VERSION_MINOR * 100 + tskKERNEL_VERSION_BUILD)
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const BaseType_t was_delayed = xTaskDelayUntil(&_previous_wake_time_ticks, taskIntervalTicks);
            if (was_delayed) {
                _was_delayed = true;
            }
#else
            vTaskDelayUntil(&_previous_wake_time_ticks, taskIntervalTicks);
#endif
        // record tick_counts for instrumentation. Not sure if this is useful anymore
        const TickType_t tick_count = xTaskGetTickCount();
        _tick_count_delta = tick_count - _tick_count_previous;
        _tick_count_previous = tick_count;
        const time_us32_t time_microseconds = time_us();
        _time_microseconds_delta = time_microseconds - _time_microseconds_previous;
        _time_microseconds_previous = time_microseconds;
        if (_time_microseconds_delta > 0) { // guard against the case of this while loop executing twice on the same tick interval
            _cms.updateCMS(_parameter_group, time_microseconds, _time_microseconds_delta);
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for CMS::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void CMS_Task::task_static(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<CMS_Task*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
