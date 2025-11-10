#include "Main.h"


#include <AHRS.h>
#include <AHRS_Task.h>
#include <BackchannelTask.h>
#include <BlackboxTask.h>
#if defined(M5_UNIFIED)
#include <ButtonsM5.h>
#endif
#include <Cockpit.h>
#include <Debug.h>
#include <FlightController.h>
#include <IMU_Filters.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <MSP_ProtoFlight.h>
#include <MSP_Task.h>
#include <NonVolatileStorage.h>
#include <OSD_Task.h>
#include <ReceiverBase.h>
#include <ReceiverTask.h>
#if defined(M5_UNIFIED)
#include <ScreenM5.h>
#endif
#include <VehicleControllerTask.h>


/*!
Protoflight setup.
This consists of creating all the Protoflight objects and then creating all the tasks.
*/
void Main::setup()
{
#if defined(M5_UNIFIED)
    // Initialize the M5Stack object
    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();
#endif

#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
    Serial.begin(115200);
#endif

    //
    // Statically allocate all the Protoflight objects
    //

    // Statically allocate and initialize nonvolatile storage
    static NonVolatileStorage nvs;
    nvs.init();
    nvs.setCurrentPidProfileIndex(nvs.loadPidProfileIndex());
    nvs.setCurrentRateProfileIndex(nvs.loadRateProfileIndex());

    // create the IMU and get its sample rate
    static IMU_Base& imuSensor = createIMU();
    const uint32_t imuSampleRateHz = imuSensor.getGyroSampleRateHz();
    std::array<char, 128> buf;
    sprintf(&buf[0], "\r\n**** IMU sample rate:%dHz\r\n\r\n", static_cast<int>(imuSampleRateHz));
    print(&buf[0]);

    // Statically allocate the debug object
    static Debug debug;
    // statically allocate the IMU_Filters
    const float AHRS_taskIntervalSeconds = 1.0F / static_cast<float>(imuSampleRateHz);
    static IMU_Filters imuFilters(MotorMixerBase::motorCount(nvs.loadMotorMixerType()), debug, AHRS_taskIntervalSeconds);
    imuFilters.setConfig(nvs.loadIMU_FiltersConfig());
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    imuFilters.setDynamicNotchFilterConfig(nvs.loadDynamicNotchFilterConfig());
#endif
    const auto AHRS_taskIntervalMicroSeconds = static_cast<uint32_t>(AHRS_taskIntervalSeconds*1000000.0F);
    static AHRS_MessageQueue AHRS_MessageQueue;
    FlightController& flightController = createFlightController(AHRS_taskIntervalMicroSeconds, AHRS_MessageQueue, imuFilters, debug, nvs);

    AHRS& ahrs = createAHRS(flightController, imuSensor, imuFilters);

    ReceiverBase& receiver = createReceiver();

    Cockpit& cockpit = createCockpit(receiver, flightController, debug, AHRS_MessageQueue, nvs);
#if defined(USE_MSP)
    MSP_SerialBase& mspSerial = createMSP(ahrs, flightController, cockpit, receiver, cockpit.getAutopilot(), debug, nvs);
#endif
#if defined(USE_BLACKBOX)
    Blackbox& blackbox = createBlackBox(ahrs, flightController, AHRS_MessageQueue, cockpit, receiver, imuFilters, debug);
#endif
#if defined(USE_OSD)
    OSD& osd = createOSD(flightController, debug);
#endif

#if defined(M5_UNIFIED)
    // Holding BtnA down while switching on enters calibration mode.
    if (M5.BtnA.isPressed()) {
        calibrateIMU(nvs, ahrs, CALIBRATE_ACC_AND_GYRO);
    }
    checkIMU_Calibration(nvs, ahrs);
    // Holding BtnC down while switching on resets the nvs.
    if (M5.BtnC.isPressed()) {
        nvs.clear();
    }
    // Statically allocate the screen.
    static ScreenM5 screen(ahrs, flightController, receiver);
    ReceiverWatcher* receiverWatcher = &screen;
    _screen = &screen;
    _screen->updateTemplate(); // Update the screen as soon as we can, to minimize the time the screen is blank
    // Statically allocate the buttons.
    static ButtonsM5 buttons(flightController, receiver, _screen);
    _buttons = &buttons;
    // Holding BtnB down while switching on initiates binding.
    // The Atom has no BtnB, so it always broadcasts address for binding on startup.
    if (M5.getBoard() ==lgfx::board_M5AtomS3 || M5.BtnB.wasPressed()) {
        receiver.broadcastMyEUI();
    }
#else
    // no buttons defined, so always broadcast address for binding on startup
    receiver.broadcastMyEUI();
    ReceiverWatcher* receiverWatcher = nullptr;
#endif // M5_UNIFIED


    //
    // Create all the tasks
    //

    // The DashboardTask runs in the main loop
    static DashboardTask dashboardTask(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS);
    _tasks.dashboardTask = &dashboardTask;
    reportDashboardTask();

    TaskBase::task_info_t taskInfo {};

#if defined(AHRS_TASK_IS_TIMER_DRIVEN)
    _tasks.ahrsTask = AHRS_Task::createTask(taskInfo, ahrs, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, AHRS_taskIntervalMicroSeconds);
#else
    _tasks.ahrsTask = AHRS_Task::createTask(taskInfo, ahrs, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, 0);
#endif
    printTaskInfo(taskInfo);

    _tasks.flightControllerTask = VehicleControllerTask::createTask(taskInfo, flightController, FC_TASK_PRIORITY, FC_TASK_CORE);
    printTaskInfo(taskInfo);

    _tasks.receiverTask = ReceiverTask::createTask(taskInfo, cockpit, receiverWatcher, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_CORE, RECEIVER_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#if defined(USE_MSP)
    _tasks.mspTask = MSP_Task::createTask(taskInfo, mspSerial, MSP_TASK_PRIORITY, MSP_TASK_CORE, MSP_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(USE_BLACKBOX)
    _tasks.blackboxTask = BlackboxTask::createTask(taskInfo, blackbox, AHRS_MessageQueue, BLACKBOX_TASK_PRIORITY, BLACKBOX_TASK_CORE, BLACKBOX_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(USE_OSD)
    _tasks.osdTask = OSD_Task::createTask(taskInfo, osd, OSD_TASK_PRIORITY, OSD_TASK_CORE, OSD_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
    BackchannelBase& backchannel = createBackchannel(flightController, ahrs, receiver, &dashboardTask, nvs);
    _tasks.backchannelTask = BackchannelTask::createTask(taskInfo, backchannel, BACKCHANNEL_TASK_PRIORITY, BACKCHANNEL_TASK_CORE, BACKCHANNEL_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
}


void Main::reportDashboardTask()
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    // The main task is set up by the framework, so just print its details.
    // It has name "loopTask" and priority 1.
    const TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    const UBaseType_t taskPriority = uxTaskPriorityGet(taskHandle);
    const char* taskName = pcTaskGetName(taskHandle);
    Serial.printf("\r\n\r\n**** Main loop task, name:'%s' priority:%d, tickRate:%dHz\r\n", taskName, static_cast<int>(taskPriority), configTICK_RATE_HZ);
#endif
}

void Main::printTaskInfo(TaskBase::task_info_t& taskInfo)
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    Serial.printf("**** %s, %.*s core:%d, priority:%d, ", taskInfo.name, 18 - strlen(taskInfo.name), "                ", static_cast<int>(taskInfo.core), static_cast<int>(taskInfo.priority));
    if (taskInfo.taskIntervalMicroseconds == 0) {
        Serial.printf("interrupt driven\r\n");
    } else {
        Serial.printf("task interval:%ums\r\n", static_cast<unsigned int>(taskInfo.taskIntervalMicroseconds / 1000));
    }
#else
    std::array<char, 128> buf;
    sprintf(&buf[0], "**** %s, %.*s core:%d, priority:%d, ", taskInfo.name, static_cast<int>(18 - strlen(taskInfo.name)), "                ", static_cast<int>(taskInfo.core), static_cast<int>(taskInfo.priority));
    printf(&buf[0]);
    if (taskInfo.taskIntervalMicroseconds == 0) {
        printf("interrupt driven\r\n");
    } else {
        sprintf(&buf[0], "task interval:%ums\r\n", static_cast<unsigned int>(taskInfo.taskIntervalMicroseconds / 1000));
        printf(&buf[0]);
    }
#endif
}

#if defined(FRAMEWORK_USE_FREERTOS)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) // NOLINT(readability-non-const-parameter)
{
    assert(false && "stack overflow");
#if defined(FRAMEWORK_ARDUINO_ESP32)
    Serial.printf("\r\n\r\n*********\r\n");
    Serial.printf("********Task '%s' stack overflow ********\r\n", pcTaskName);
    Serial.printf("*********\r\n\r\n");
#else
    (void)xTask;
    (void)pcTaskName;
#endif
}
#endif

void Main::print(const char* buf)
{
#if defined(FRAMEWORK_RPI_PICO)
    printf(&buf[0]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
#elif defined(FRAMEWORK_ESPIDF)
    (void)buf;
#elif defined(FRAMEWORK_STM32_CUBE)
    (void)buf;
#elif defined(FRAMEWORK_TEST)
    printf(&buf[0]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
#else
    Serial.print(&buf[0]);
#endif
}

/*!
The dashboard loop handles:
1. Output to the screen
2. Input from the buttons
*/
void DashboardTask::loop()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    const TickType_t tickCount = xTaskGetTickCount();
    _tickCountDelta = tickCount - _tickCountPrevious;
    _tickCountPrevious = tickCount;
#endif // USE_FREERTOS
}

void Main::loop() // NOLINT(readability-make-member-function-const)
{
#if defined(FRAMEWORK_USE_FREERTOS)
    vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS / 1000));
    [[maybe_unused]] const TickType_t tickCount = xTaskGetTickCount();
#else
    // simple round-robbin scheduling
    _tasks.dashboardTask->loop();
    _tasks.ahrsTask->loop();
    _tasks.flightControllerTask->loop();
    _tasks.receiverTask->loop();
    [[maybe_unused]] const uint32_t tickCount = timeUs() / 1000;
#endif

#if defined(USE_SCREEN)
    // screen and button update tick counts are coprime, so screen and buttons are not normally updated in same loop
    // update the screen every 101 ticks (0.1 seconds)
    if (_screenTickCount - tickCount > 101) {
        _screenTickCount = tickCount;
        _screen->update();
    }
#endif
#if defined(USE_BUTTONS)
    // update the buttons every 149 ticks (0.15 seconds)
    if (_buttonsTickCount - tickCount > 149) {
        _buttonsTickCount = tickCount;
        _buttons->update();
    }
#endif
}
