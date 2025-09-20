#include "Main.h"



#if defined(FRAMEWORK_RPI_PICO)
int main()
{
    static Main mainTask;
    mainTask.setup();
    while (true) {
        mainTask.loop();
    }
}

#elif defined(FRAMEWORK_ESPIDF)

extern "C" void app_main()
{
    static Main mainTask;
    mainTask.setup();
    while (true) {
        mainTask.loop();
    }
}

#elif defined(FRAMEWORK_STM32_CUBE)

int main()
{
    static Main mainTask;
    mainTask.setup();
    while (true) {
        mainTask.loop();
    }
}

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO

#include <Arduino.h>

namespace { // use anonymous namespace to make items local to this translation unit
    Main* mainTask;
} // end namespace

void setup()// cppcheck-suppress unusedFunction
{
    static Main mainTaskStatic;
    mainTask = &mainTaskStatic;
    mainTask->setup();
}


void loop()// cppcheck-suppress unusedFunction
{
    mainTask->loop();
}

#endif
