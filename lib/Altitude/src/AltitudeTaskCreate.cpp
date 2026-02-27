#include "AltitudeTask.h"

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


AltitudeTask* AltitudeTask::create_task(const parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds)
{
    task_info_t task_info {};
    return create_task(task_info, parameters, priority, core, task_interval_microseconds);
}

AltitudeTask* AltitudeTask::create_task(task_info_t& task_info, const parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static AltitudeTask altitude_task(task_interval_microseconds, parameters);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t task_parameters { // NOLINT(misc-const-correctness) false positive
        .task = &altitude_task
    };
#if !defined(ALTITUDE_TASK_STACK_DEPTH_BYTES)
    enum { ALTITUDE_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array <uint8_t, ALTITUDE_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, ALTITUDE_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    task_info = {
        .task_handle = nullptr,
        .name = "AltitudeTask",
        .stack_depth_bytes = ALTITUDE_TASK_STACK_DEPTH_BYTES,
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
    static StaticTask_t task_buffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    task_info.task_handle = xTaskCreateStaticPinnedToCore(
        AltitudeTask::Task,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create Altitude task");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    task_info.task_handle = xTaskCreateStaticAffinitySet(
        AltitudeTask::Task,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create Altitude task");
#else
    task_info.task_handle = xTaskCreateStatic(
        AltitudeTask::Task,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer
    );
    assert(task_info.task_handle != nullptr && "Unable to create Altitude task");
    // vTaskCoreAffinitySet(task_info.task_handle, task_info.core);
#endif
#else
    (void)task_parameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &altitude_task;
}
