#include "ahrs_task.h"

#include <ahrs.h>
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


AhrsTask* AhrsTask::create_task(const ahrs_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds)
{
    task_info_t task_info {};
    return create_task(task_info, context, priority, core, task_interval_microseconds);
}

AhrsTask* AhrsTask::create_task(task_info_t& task_info, const ahrs_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static AhrsTask ahrs_task(task_interval_microseconds, context);
    context.ahrs.set_task(&ahrs_task);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &ahrs_task
    };
#if !defined(AHRS_TASK_STACK_DEPTH_BYTES)
    enum { AHRS_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array <uint8_t, AHRS_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, AHRS_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    task_info = {
        .task_handle = nullptr,
        .name = "AhrsTask",
        .stack_depth_bytes = AHRS_TASK_STACK_DEPTH_BYTES,
        .stack_buffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .task_interval_microseconds = task_interval_microseconds,
    };

#if defined(FRAMEWORK_USE_FREERTOS)
    assert(std::strlen(task_info.name) < configMAX_TASK_NAME_LEN);
    assert(task_info.priority < configMAX_PRIORITIES);

#if !defined(configCHECK_FOR_STACK_OVERFLOW)
    // fill the stack so we can do our own stack overflow detection
    enum { STACK_FILLER = 0xA5 };
    stack.fill(STACK_FILLER);
#endif
    static StaticTask_t taskBuffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    task_info.task_handle = xTaskCreateStaticPinnedToCore(
        AhrsTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &taskParameters,
        task_info.priority,
        &stack[0],
        &taskBuffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create AHRS task");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    task_info.task_handle = xTaskCreateStaticAffinitySet(
        AhrsTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &taskParameters,
        task_info.priority,
        &stack[0],
        &taskBuffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create AHRS task");
#else
    task_info.task_handle = xTaskCreateStatic(
        AhrsTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &taskParameters,
        task_info.priority,
        &stack[0],
        &taskBuffer
    );
    assert(task_info.task_handle != nullptr && "Unable to create AHRS task");
    // vTaskCoreAffinitySet(task_info.task_handle, task_info.core);
#endif
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &ahrs_task;
}
