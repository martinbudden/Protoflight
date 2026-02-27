#include "OSD.h"
#include "OSD_Task.h"

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


OSD_Task* OSD_Task::create_task(OSD& osd, osd_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds)
{
    task_info_t task_info {};
    return create_task(task_info, osd, parameter_group, priority, core, task_interval_microseconds);
}

OSD_Task* OSD_Task::create_task(task_info_t& task_info, OSD& osd, osd_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static OSD_Task osd_task(task_interval_microseconds, osd, parameter_group);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t task_parameters { // NOLINT(misc-const-correctness) false positive
        .task = &osd_task
    };
#if !defined(OSD_TASK_STACK_DEPTH_BYTES)
    enum { OSD_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array <uint8_t, OSD_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, OSD_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    task_info = {
        .task_handle = nullptr,
        .name = "OSD_Task",
        .stack_depth_bytes = OSD_TASK_STACK_DEPTH_BYTES,
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
        OSD_Task::Task,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create OSD task");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    task_info.task_handle = xTaskCreateStaticAffinitySet(
        OSD_Task::Task,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create OSD task");
#else
    task_info.task_handle = xTaskCreateStatic(
        OSD_Task::Task,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer
    );
    assert(task_info.task_handle != nullptr && "Unable to create OSD task");
    // vTaskCoreAffinitySet(task_info.task_handle, task_info.core);
#endif
#else
    (void)task_parameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &osd_task;
}
