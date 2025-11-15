#include "Dashboard.h"
#include "DashboardTask.h"

#include <array>
#include <cassert>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif
#endif


DashboardTask* DashboardTask::createTask(Dashboard& dashboard, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds)
{
    task_info_t taskInfo {};
    return createTask(taskInfo, dashboard, priority, core, taskIntervalMicroseconds);
}

DashboardTask* DashboardTask::createTask(task_info_t& taskInfo, Dashboard& dashboard, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static DashboardTask dashboardTask(taskIntervalMicroseconds, dashboard);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &dashboardTask
    };
#if !defined(DashboardTASK_STACK_DEPTH_BYTES)
    enum { DashboardTASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array <uint8_t, DashboardTASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, DashboardTASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    taskInfo = {
        .taskHandle = nullptr,
        .name = "DashboardTask",
        .stackDepthBytes = DashboardTASK_STACK_DEPTH_BYTES,
        .stackBuffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .taskIntervalMicroseconds = taskIntervalMicroseconds,
    };

#if defined(FRAMEWORK_USE_FREERTOS)
    assert(std::strlen(taskInfo.name) < configMAX_TASK_NAME_LEN);
    assert(taskInfo.priority < configMAX_PRIORITIES);

#if !defined(configCHECK_FOR_STACK_OVERFLOW)
    // fill the stack so we can do our own stack overflow detection
    enum { STACK_FILLER = 0xA5 };
    stack.fill(STACK_FILLER);
#endif
    static StaticTask_t taskBuffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        DashboardTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create Dashboard task");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    taskInfo.taskHandle = xTaskCreateStaticAffinitySet(
        DashboardTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create Dashboard task");
#else
    taskInfo.taskHandle = xTaskCreateStatic(
        DashboardTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create Dashboard task");
    // vTaskCoreAffinitySet(taskInfo.taskHandle, taskInfo.core);
#endif
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &dashboardTask;
}
