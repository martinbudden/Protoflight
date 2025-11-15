#include "Main.h"

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


#if defined(FRAMEWORK_RPI_PICO)

int main()
{
    static Main mainTask;
    mainTask.setup();
#if defined(FRAMEWORK_USE_FREERTOS)
    vTaskDelete(nullptr); // Deletes the current task (loop task)
#else
    while (true) {
        mainTask.loop();
    }
#endif
}

#elif defined(FRAMEWORK_ESPIDF)

extern "C" void app_main()
{
    static Main mainTask;
    mainTask.setup();
#if defined(FRAMEWORK_USE_FREERTOS)
    vTaskDelete(nullptr); // Deletes the current task (loop task)
#else
    while (true) {
        mainTask.loop();
    }
#endif
}

#elif defined(FRAMEWORK_STM32_CUBE)

int main()
{
    static Main mainTask;
    mainTask.setup();
#if defined(FRAMEWORK_USE_FREERTOS)
    vTaskDelete(nullptr); // Deletes the current task (loop task)
#else
    while (true) {
        mainTask.loop();
    }
#endif
}

#elif defined(FRAMEWORK_TEST)

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    static Main mainTask;
    mainTask.setup();
#if defined(FRAMEWORK_USE_FREERTOS)
    vTaskDelete(nullptr); // Deletes the current task (loop task)
#else
    while (true) {
        mainTask.loop();
    }
#endif

    return 0;
}

#else // defaults to FRAMEWORK_ARDUINO

#include <Arduino.h>

namespace { // use anonymous namespace to make items local to this translation unit
    Main* mainTask;
} // end namespace

void setup() // cppcheck-suppress unusedFunction
{
    static Main mainTaskStatic;
    mainTask = &mainTaskStatic;
    mainTask->setup();
#if defined(FRAMEWORK_USE_FREERTOS)
    vTaskDelete(nullptr); // Deletes the current task (loop task)
#endif
}


void loop() // cppcheck-suppress unusedFunction
{
#if !defined(FRAMEWORK_USE_FREERTOS)
    mainTask->loop();
#endif
}

#endif
