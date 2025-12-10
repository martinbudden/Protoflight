#include "Main.h"

#include "CMS_Task.h"
#include "Cockpit.h"
#include "DashboardTask.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "MSP_Protoflight.h"
#include "MSP_Serial.h"
#include "NonVolatileStorage.h"
#include "OSD_Task.h"

#include <AHRS.h>
#include <AHRS_Task.h>
#include <BackchannelTask.h>
#include <BlackboxTask.h>
#include <Debug.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <MSP_Task.h>
#include <ReceiverBase.h>
#include <ReceiverTask.h>
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
#if defined(M5_UNIFIED)
    // Holding BtnC down while switching on resets the nvs.
    if (M5.BtnC.isPressed()) {
        nvs.clear();
    }
#endif
    nvs.setCurrentPidProfileIndex(nvs.loadPidProfileIndex());
    nvs.setCurrentRateProfileIndex(nvs.loadRateProfileIndex());

#if defined(FRAMEWORK_ARDUINO) || defined(FRAMEWORK_ARDUINO_ESP32)
    delay(500); // delay to allow serial port to initialize before first print
#endif

    // create the IMU and get its sample rate
    static IMU_Base& imuSensor = createIMU(nvs);
    const float AHRS_taskIntervalSeconds = 1.0F / static_cast<float>(imuSensor.getGyroSampleRateHz());

    static Debug debug;

    FlightController& flightController = createFlightController(AHRS_taskIntervalSeconds, debug, nvs);

    IMU_Filters& imuFilters = createIMU_Filters(AHRS_taskIntervalSeconds, flightController.getMotorMixer(), debug, nvs); 

    AHRS& ahrs = createAHRS(flightController, imuSensor, imuFilters);

    ReceiverBase& receiver = createReceiver(nvs);

    Cockpit& cockpit = createCockpit(receiver, flightController, debug, imuFilters, nvs);

    // create the optional components according to build flags
    Blackbox* blackbox = createBlackBox(ahrs, flightController, cockpit, receiver, imuFilters, debug);
    DisplayPortBase& displayPort = createDisplayPort(debug);
    OSD* osd = createOSD(displayPort, flightController, cockpit, debug, nvs);
    VTX* vtx = createVTX(nvs); // VTX settings may be changed by MSP or the CMS (also by CLI when it gets implemented).
    [[maybe_unused]] MSP_Serial* mspSerial = createMSP(ahrs, flightController, cockpit, receiver, cockpit.getAutopilot(), imuFilters, debug, nvs, blackbox, vtx, osd);
    [[maybe_unused]] CMS* cms = createCMS(displayPort, receiver, cockpit, imuFilters, imuSensor, osd, vtx);
    [[maybe_unused]] Dashboard* dashboard = createDashboard(displayPort, ahrs, flightController, receiver);


    //
    // Create all the tasks
    //


    TaskBase::task_info_t taskInfo {};

#if defined(AHRS_TASK_IS_TIMER_DRIVEN)
    const auto AHRS_taskIntervalMicroseconds = static_cast<uint32_t>(AHRS_taskIntervalSeconds*1000000.0F);
    _tasks.ahrsTask = AHRS_Task::createTask(taskInfo, ahrs, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, AHRS_taskIntervalMicroseconds);
#else
    _tasks.ahrsTask = AHRS_Task::createTask(taskInfo, ahrs, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, 0);
#endif
    printTaskInfo(taskInfo);

    _tasks.flightControllerTask = VehicleControllerTask::createTask(taskInfo, flightController, FC_TASK_PRIORITY, FC_TASK_CORE);
    printTaskInfo(taskInfo);

    _tasks.receiverTask = ReceiverTask::createTask(taskInfo, receiver, cockpit, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_CORE, RECEIVER_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#if defined(USE_DASHBOARD)
    assert(dashboard != nullptr);
    _tasks.dashboardTask = DashboardTask::createTask(taskInfo, *dashboard, DASHBOARD_TASK_PRIORITY, DASHBOARD_TASK_CORE, DASHBOARD_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(USE_MSP)
    assert(mspSerial != nullptr);
    _tasks.mspTask = MSP_Task::createTask(taskInfo, *mspSerial, MSP_TASK_PRIORITY, MSP_TASK_CORE, MSP_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(USE_BLACKBOX)
    assert(blackbox != nullptr);
    _tasks.blackboxTask = BlackboxTask::createTask(taskInfo, *blackbox, flightController.getAHRS_MessageQueue(), BLACKBOX_TASK_PRIORITY, BLACKBOX_TASK_CORE, BLACKBOX_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(USE_OSD)
    const uint32_t osdTaskIntervalMicroseconds = 1'000'000 / osd->getConfig().framerate_hz;
    assert(osd != nullptr);
    _tasks.osdTask = OSD_Task::createTask(taskInfo, *osd, OSD_TASK_PRIORITY, OSD_TASK_CORE, osdTaskIntervalMicroseconds);
    printTaskInfo(taskInfo);
#endif
#if defined(USE_CMS)
    assert(cms != nullptr);
    _tasks.cmsTask = CMS_Task::createTask(taskInfo, *cms, CMS_TASK_PRIORITY, CMS_TASK_CORE, CMS_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
    BackchannelBase& backchannel = createBackchannel(flightController, ahrs, receiver, nvs, _tasks.dashboardTask);
    _tasks.backchannelTask = BackchannelTask::createTask(taskInfo, backchannel, BACKCHANNEL_TASK_PRIORITY, BACKCHANNEL_TASK_CORE, BACKCHANNEL_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
}


void Main::printTaskInfo(TaskBase::task_info_t& taskInfo)
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    Serial.printf("**** %s, %.*s core:%d, priority:%d, ", taskInfo.name, 18 - strlen(taskInfo.name), "                ", static_cast<int>(taskInfo.core), static_cast<int>(taskInfo.priority));
    if (taskInfo.taskIntervalMicroseconds == 0) {
        Serial.printf("interrupt driven\r\n");
    } else {
        if (taskInfo.taskIntervalMicroseconds / 1000 == 0) {
            Serial.printf("task interval:%4uus\r\n", static_cast<unsigned int>(taskInfo.taskIntervalMicroseconds));
        } else {
            Serial.printf("task interval:%3ums\r\n", static_cast<unsigned int>(taskInfo.taskIntervalMicroseconds / 1000));
        }
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
