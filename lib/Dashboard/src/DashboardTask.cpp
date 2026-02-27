#include "Dashboard.h"
#include "DashboardTask.h"

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
void DashboardTask::loop()
{
    const time_us32_t time_microseconds = time_us();
    _tick_count_delta = time_microseconds - _time_microseconds_previous;

    if (_time_microseconds_delta >= _task_interval_microseconds) { // if _task_interval_microseconds has passed, then run the update
        _time_microseconds_previous = time_microseconds;
        _dashboard.updateDashboard(_parameter_group);
    }
}

/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
[[noreturn]] void DashboardTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    const uint32_t taskIntervalTicks = _task_interval_microseconds < 1000 ? 1 : pdMS_TO_TICKS(_task_interval_microseconds / 1000);
    _previous_wake_time_ticks = xTaskGetTickCount();
    while (true) {
        // delay until the end of the next taskIntervalTicks
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const BaseType_t was_delayed = xTaskDelayUntil(&_previous_wake_time_ticks, taskIntervalTicks);
            if (was_delayed) {
                _was_delayed = true;
            }
#else
            vTaskDelayUntil(&_previous_wake_time_ticks, taskIntervalTicks);
#endif
        _dashboard.updateDashboard(_parameter_group);
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for AHRS::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void DashboardTask::task_static(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<DashboardTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
