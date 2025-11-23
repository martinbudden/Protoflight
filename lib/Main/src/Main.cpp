#include "Main.h"

#include <AHRS.h>
#include <AHRS_Task.h>
#include <BackchannelTask.h>
#include <BlackboxTask.h>
#if defined(M5_UNIFIED)
#include <ButtonsM5.h>
#endif
#include <CMS_Task.h>
#include <Cockpit.h>
#include <Dashboard.h>
#include <DashboardTask.h>
#include <Debug.h>
#include <DisplayPortM5GFX.h>
#include <DisplayPortMax7456.h>
#include <DisplayPortNull.h>
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
    const float AHRS_taskIntervalSeconds = 1.0F / static_cast<float>(imuSampleRateHz);

    std::array<char, 128> buf;
    sprintf(&buf[0], "\r\n**** IMU sample rate:%dHz\r\n\r\n", static_cast<int>(imuSampleRateHz));
    print(&buf[0]);

    static Debug debug;

    FlightController& flightController = createFlightController(AHRS_taskIntervalSeconds, debug, nvs);

    IMU_Filters& imuFilters = createIMU_Filters(AHRS_taskIntervalSeconds, flightController.getMotorMixer(), debug, nvs); 

    AHRS& ahrs = createAHRS(flightController, imuSensor, imuFilters);

    ReceiverBase& receiver = createReceiver();

    Cockpit& cockpit = createCockpit(receiver, flightController, debug, imuFilters, nvs);
#if defined(USE_MSP)
    MSP_SerialBase& mspSerial = createMSP(ahrs, flightController, cockpit, receiver, cockpit.getAutopilot(), imuFilters, debug, nvs);
#endif
#if defined(USE_BLACKBOX)
    Blackbox& blackbox = createBlackBox(ahrs, flightController, cockpit, receiver, imuFilters, debug);
#endif

#if defined(M5_UNIFIED)
    // Holding BtnA down while switching on enters calibration mode.
    if (M5.BtnA.isPressed()) {
        calibrateIMUandSave(nvs, ahrs.getIMU(), IMU_Base::CALIBRATE_ACC_AND_GYRO);
    }
    checkIMU_Calibration(nvs, ahrs.getIMU());
    // Holding BtnC down while switching on resets the nvs.
    if (M5.BtnC.isPressed()) {
        nvs.clear();
    }
    static M5Canvas canvas(&M5.Display);
    static DisplayPortM5GFX displayPort(canvas, 320, 240);

    // Statically allocate the screen.
    static ScreenM5 screen(displayPort, ahrs, flightController, receiver);
#if defined(USE_DASHBOARD)
    screen.updateTemplate(); // Update the screen as soon as we can, to minimize the time the screen is blank
#endif
    // Statically allocate the buttons.
    static ButtonsM5 buttons(flightController, receiver, &screen);
    // Holding BtnB down while switching on initiates binding.
    // The Atom has no BtnB, so it always broadcasts address for binding on startup.
    if (M5.getBoard() ==lgfx::board_M5AtomS3 || M5.BtnB.wasPressed()) {
        receiver.broadcastMyEUI();
    }
    ReceiverWatcher* receiverWatcher = &screen;
#if defined(USE_DASHBOARD)
    static Dashboard dashboard(&screen, &buttons);
#else
    _screen = &screen;
    _buttons = &buttons;
#endif
#else
#if defined(USE_MAX7456)
    [[maybe_unused]] static DisplayPortMax7456 displayPort(BUS_SPI::MAX7456_SPI_INDEX, BUS_SPI::MAX7456_SPI_PINS, debug);
#else
    [[maybe_unused]] static DisplayPortNull displayPort;
#endif
    // no buttons defined, so always broadcast address for binding on startup
    receiver.broadcastMyEUI();
    ReceiverWatcher* receiverWatcher = nullptr;
#endif // M5_UNIFIED

#if defined(USE_OSD)
    OSD& osd = createOSD(displayPort, flightController, cockpit, debug, nvs);
#endif
#if defined(USE_CMS)
#if defined(USE_OSD)
    CMS& cms = createCMS(displayPort, receiver, cockpit, imuFilters, ahrs.getIMU(), nvs, &osd);
#else
    CMS& cms = createCMS(displayPort, receiver, cockpit, imuFilters, ahrs.getIMU(), nvs, nullptr);
#endif
#endif


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

    _tasks.receiverTask = ReceiverTask::createTask(taskInfo, cockpit, receiverWatcher, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_CORE, RECEIVER_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#if defined(USE_DASHBOARD)
    _tasks.dashboardTask = DashboardTask::createTask(taskInfo, dashboard, DASHBOARD_TASK_PRIORITY, DASHBOARD_TASK_CORE, DASHBOARD_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(USE_MSP)
    _tasks.mspTask = MSP_Task::createTask(taskInfo, mspSerial, MSP_TASK_PRIORITY, MSP_TASK_CORE, MSP_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(USE_BLACKBOX)
    _tasks.blackboxTask = BlackboxTask::createTask(taskInfo, blackbox, flightController.getAHRS_MessageQueue(), BLACKBOX_TASK_PRIORITY, BLACKBOX_TASK_CORE, BLACKBOX_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(USE_OSD)
    const uint32_t osdTaskIntervalMicroseconds = 1'000'000 / osd.getConfig().framerate_hz;
    _tasks.osdTask = OSD_Task::createTask(taskInfo, osd, OSD_TASK_PRIORITY, OSD_TASK_CORE, osdTaskIntervalMicroseconds);
    printTaskInfo(taskInfo);
#endif
#if defined(USE_CMS)
    _tasks.cmsTask = CMS_Task::createTask(taskInfo, cms, CMS_TASK_PRIORITY, CMS_TASK_CORE, CMS_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(taskInfo);
#endif
#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
    BackchannelBase& backchannel = createBackchannel(flightController, ahrs, receiver, _tasks.dashboardTask, nvs);
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

void Main::loop() // NOLINT(readability-make-member-function-const)
{
#if !defined(FRAMEWORK_USE_FREERTOS)
    // simple round-robbin scheduling
    _tasks.ahrsTask->loop();
    _tasks.flightControllerTask->loop();
    _tasks.receiverTask->loop();
    [[maybe_unused]] const uint32_t tickCount = timeUs() / 1000;

#if defined(USE_DASHBOARD)
    _tasks.dashboardTask->loop();
#else
#if defined(USE_SCREEN)
    // screen and button update tick counts are coprime, so screen and buttons are not normally updated in same loop
    // update the screen every 101 ticks (0.1 seconds)
    if (tickCount - _screenTickCount > 101) {
        _screenTickCount = tickCount;
        _screen->update();
    }
#endif
#if defined(USE_BUTTONS)
    // update the buttons every 149 ticks (0.15 seconds)
    if (tickCount - _buttonsTickCount > 149) {
        _buttonsTickCount = tickCount;
        _buttons->update();
    }
#endif
#endif // USE_DASHBOARD
#endif // FRAMEWORK_USE_FREERTOS
}
