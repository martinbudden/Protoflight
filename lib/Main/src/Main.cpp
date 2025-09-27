#include "Main.h"


#include <AHRS.h>
#include <AHRS_Task.h>
#include <BackchannelFlightController.h>
#include <BackchannelTask.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <BackchannelTransceiverESPNOW.h>
#endif
#include <BlackboxCallbacks.h>
#include <BlackboxMessageQueueAHRS.h>
#include <BlackboxProtoFlight.h>
#include <BlackboxSerialDeviceSDCard.h>
#include <BlackboxTask.h>
#if defined(M5_UNIFIED)
#include <ButtonsM5.h>
#endif
#include <Debug.h>
#include <Features.h>
#include <FlightController.h>
#include <IMU_Filters.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <MSP_ProtoFlight.h>
#include <MSP_Serial.h>
#include <MSP_Task.h>
#include <MotorMixerQuadX_DShot.h>
#include <MotorMixerQuadX_DShotBitbang.h>
#include <MotorMixerQuadX_PWM.h>
#include <NonVolatileStorage.h>
#include <RPM_Filters.h>
#include <RadioController.h>
#include <ReceiverAtomJoyStick.h>
#include <ReceiverNull.h>
#include <ReceiverSBUS.h>
#include <ReceiverTask.h>
#if defined(M5_UNIFIED)
#include <ScreenM5.h>
#endif
#include <TimeMicroseconds.h>
#include <VehicleControllerTask.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <WiFi.h>
#endif


/*!
Setup for the main loop, motor control task, and AHRS(Attitude and Heading Reference System) task.
*/
void Main::setup()
{
// NOLINTBEGIN(misc-const-correctness)
#if defined(M5_UNIFIED)
    // Initialize the M5Stack object
    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();
#endif

#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_STM32_CUBE)
    Serial.begin(115200);
#endif

    // Statically allocate the debug object
    static Debug debug;
    // Statically allocate and initialize nonvolatile storage
    static NonVolatileStorage nvs;
    nvs.init();
    const uint8_t currentRateProfile = nvs.RateProfileIndexLoad();
    const uint8_t currentPID_Profile = nvs.PidProfileIndexLoad();
    nvs.setCurrentPidProfileIndex(currentPID_Profile);

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
    // Set WiFi to station mode
    WiFi.mode(WIFI_STA);
    // Disconnect from Access Point if it was previously connected
    WiFi.disconnect();
    // get my MAC address
    uint8_t myMacAddress[ESP_NOW_ETH_ALEN];
    WiFi.macAddress(&myMacAddress[0]);
#if !defined(RECEIVER_CHANNEL)
    enum { RECEIVER_CHANNEL = 3 };
#endif
    static ReceiverAtomJoyStick receiver(&myMacAddress[0], RECEIVER_CHANNEL);
    const esp_err_t espErr = receiver.init();
    Serial.printf("\r\n\r\n**** ESP-NOW Ready:%X\r\n\r\n", espErr);
    assert(espErr == ESP_OK && "Unable to setup receiver.");
#elif defined(USE_RECEIVER_SBUS)
    static ReceiverSBUS receiver(ReceiverSerial::RECEIVER_PINS, RECEIVER_UART_INDEX, ReceiverSBUS::BAUD_RATE);
#else
    static ReceiverNull receiver;
#endif

    static RadioController radioController(receiver, nvs.RadioControllerRatesLoad(currentRateProfile));

    // create the IMU and get its sample rate
#if defined(USE_IMU_BMI270_I2C) || defined(USE_IMU_BMI270_SPI)
    int32_t imuSampleRateHz = 3200; // set max sample rate for BMI270
#else
    int32_t imuSampleRateHz = 1000000 / AHRS_TASK_INTERVAL_MICROSECONDS;
#endif
    static IMU_Base& imuSensor = createIMU(imuSampleRateHz); // note, the actual set sampleRate is returned in imuSampleRateHz
#if defined(USE_AHRS_TASK_INTERRUPT_DRIVEN_SCHEDULING)
    // if the AHRS is interrupt driven, then set its task interval based on the IMU sample rate
    const uint32_t AHRS_taskIntervalMicroseconds = 1000000 / imuSampleRateHz;
#else
    const uint32_t AHRS_taskIntervalMicroseconds = AHRS_TASK_INTERVAL_MICROSECONDS;
#endif
#if defined(FRAMEWORK_RPI_PICO)
    printf("\r\n**** AHRS_taskIntervalMicroseconds:%u, IMU sample rate:%dHz\r\n\r\n", static_cast<unsigned int>(AHRS_taskIntervalMicroseconds), static_cast<int>(imuSampleRateHz));
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#else
    Serial.printf("\r\n**** AHRS_taskIntervalMicroseconds:%u, IMU sample rate:%dHz\r\n\r\n", AHRS_taskIntervalMicroseconds, imuSampleRateHz);
#endif

    // Statically allocate the MotorMixer object as defined by the build flags.
#if defined(USE_MOTOR_MIXER_QUAD_X_PWM)
    static MotorMixerQuadX_PWM motorMixer(debug, MotorMixerQuadBase::MOTOR_PINS);
#elif defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)
    enum { MOTOR_COUNT = 4 };
    static RPM_Filters rpmFilters(MOTOR_COUNT, AHRS_TASK_INTERVAL_MICROSECONDS);
    static DynamicIdleController dynamicIdleController(nvs.DynamicIdleControllerConfigLoad(currentPID_Profile), AHRS_taskIntervalMicroseconds / FC_TASK_DENOMINATOR, debug);
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    static MotorMixerQuadX_DShotBitbang motorMixer(debug, MotorMixerQuadBase::MOTOR_PINS, rpmFilters, dynamicIdleController);
#else
    static MotorMixerQuadX_DShot motorMixer(debug, MotorMixerQuadBase::MOTOR_PINS, rpmFilters, dynamicIdleController);
#endif
#if defined(USE_DYNAMIC_IDLE)
    motorMixer.setMotorOutputMin(0.0F);
#else
    motorMixer.setMotorOutputMin(0.055F); // 5.5%
#endif
#else
    static_assert(false && "MotorMixer not specified");
#endif // USE_MOTOR_MIXER

    // statically allocate the IMU_Filters
    static IMU_Filters imuFilters(motorMixer, AHRS_taskIntervalMicroseconds);
    imuFilters.setConfig(nvs.ImuFiltersConfigLoad());
#if defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)
    imuFilters.setRPM_Filters(&rpmFilters);
#endif

    // Statically allocate the AHRS
    AHRS& ahrs = createAHRS(AHRS_taskIntervalMicroseconds, imuSensor, imuFilters);

    // Statically allocate the flightController.
    static FlightController flightController(FC_TASK_DENOMINATOR, ahrs, motorMixer, radioController, debug);
    loadPID_ProfileFromNonVolatileStorage(nvs, flightController, currentPID_Profile);

    ahrs.setVehicleController(&flightController);
    radioController.setFlightController(&flightController);

    // Statically allocate the Blackbox and associated objects
#if defined(USE_BLACKBOX) || defined(USE_BLACKBOX_DEBUG)
    static BlackboxMessageQueue         blackboxMessageQueue;
    static BlackboxCallbacks            blackboxCallbacks(blackboxMessageQueue, ahrs, flightController, radioController, receiver, debug);
    static BlackboxSerialDeviceSDCard   blackboxSerialDevice(BlackboxSerialDeviceSDCard::SDCARD_SPI_PINS);
    static BlackboxProtoFlight          blackbox(blackboxCallbacks, blackboxMessageQueue, blackboxSerialDevice, flightController, radioController, imuFilters);

    static BlackboxMessageQueueAHRS     blackboxMessageQueueAHRS(blackboxMessageQueue);
    ahrs.setMessageQueue(&blackboxMessageQueueAHRS);

    flightController.setBlackbox(blackbox);
    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        //.device = Blackbox::DEVICE_NONE,
        .mode = Blackbox::MODE_NORMAL // logging starts on arming, file is saved when disarmed
        //.mode = Blackbox::MODE_ALWAYS_ON
    });
#if defined(USE_BLACKBOX_DEBUG)
    testBlackbox(blackbox, ahrs, receiver, debug);
#endif
#endif

#if defined(M5_UNIFIED)
    // Holding BtnA down while switching on enters calibration mode.
    //if (M5.BtnA.isPressed()) {
    //    calibrateGyro(*ahrs, nvs, CALIBRATE_ACC_AND_GYRO);
    //}
    //!!checkGyroCalibration(nvs, ahrs);
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

#if defined(M5_ATOM)
    // The Atom has no BtnB, so it always broadcasts address for binding on startup.
    receiver.broadcastMyEUI();
#else
    // Holding BtnB down while switching on initiates binding.
    if (M5.BtnB.wasPressed()) {
        receiver.broadcastMyEUI();
    }
#endif

#else
    // no buttons defined, so always broadcast address for binding on startup
    receiver.broadcastMyEUI();
    ReceiverWatcher* receiverWatcher = nullptr;
#endif // M5_UNIFIED

    // And finally create the AHRS, FlightController, Receiver, and MSP tasks.
    static MainTask mainTask(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS);
    _tasks.mainTask = &mainTask;
    reportMainTask();

    TaskBase::task_info_t taskInfo {};

    _tasks.ahrsTask = AHRS_Task::createTask(taskInfo, ahrs, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, AHRS_taskIntervalMicroseconds);
    taskInfo.taskIntervalMicroseconds = AHRS_taskIntervalMicroseconds;
    printTaskInfo(taskInfo);

    _tasks.flightControllerTask = VehicleControllerTask::createTask(taskInfo, flightController, FC_TASK_PRIORITY, FC_TASK_CORE);
    taskInfo.taskIntervalMicroseconds = 0;
    printTaskInfo(taskInfo);

    _tasks.receiverTask = ReceiverTask::createTask(taskInfo, receiver, radioController, receiverWatcher, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_CORE, RECEIVER_TASK_INTERVAL_MICROSECONDS);
    taskInfo.taskIntervalMicroseconds = RECEIVER_TASK_INTERVAL_MICROSECONDS;
    printTaskInfo(taskInfo);
#if defined(USE_MSP)
    // Statically allocate the MSP and associated objects
    static Features features;
    static MSP_ProtoFlight mspProtoFlight(nvs, features, ahrs, flightController, radioController, receiver, debug);
    static MSP_Stream mspStream(mspProtoFlight);
    static MSP_Serial mspSerial(mspStream);
    _tasks.mspTask = MSP_Task::createTask(taskInfo, mspSerial, MSP_TASK_PRIORITY, MSP_TASK_CORE, MSP_TASK_INTERVAL_MICROSECONDS);
    taskInfo.taskIntervalMicroseconds = MSP_TASK_INTERVAL_MICROSECONDS;
    printTaskInfo(taskInfo);
#endif
#if defined(USE_BLACKBOX)
    blackboxCallbacks.setUseMessageQueue(true);
    _tasks.blackboxTask = BlackboxTask::createTask(taskInfo, blackbox, BLACKBOX_TASK_PRIORITY, BLACKBOX_TASK_CORE, BLACKBOX_TASK_INTERVAL_MICROSECONDS);
    taskInfo.taskIntervalMicroseconds = BLACKBOX_TASK_INTERVAL_MICROSECONDS;
    printTaskInfo(taskInfo);
    //vTaskResume(taskInfo.taskHandle);
#endif

#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
    // statically allocate an MSP object
    // static MSP_ProtoFlight mspProtoFlightBackchannel(features, ahrs, flightController, radioController, receiver);
    // Statically allocate the backchannel.
    constexpr uint8_t backchannelMacAddress[ESP_NOW_ETH_ALEN] BACKCHANNEL_MAC_ADDRESS;
    static BackchannelTransceiverESPNOW backchannelTransceiverESPNOW(receiver.getESPNOW_Transceiver(), &backchannelMacAddress[0]);
    static BackchannelFlightController backchannel(
        backchannelTransceiverESPNOW,
        &backchannelMacAddress[0],
        &myMacAddress[0],
        flightController,
        ahrs,
        receiver,
        &mainTask,
        nvs
    );

    _tasks.backchannelTask = BackchannelTask::createTask(taskInfo, backchannel, BACKCHANNEL_TASK_PRIORITY, BACKCHANNEL_TASK_CORE, BACKCHANNEL_TASK_INTERVAL_MICROSECONDS);
    taskInfo.taskIntervalMicroseconds = BACKCHANNEL_TASK_INTERVAL_MICROSECONDS;
    printTaskInfo(taskInfo);
#endif
}

void Main::testBlackbox(Blackbox& blackbox, AHRS& ahrs, ReceiverBase& receiver, const Debug& debug)
{
    static uint32_t timeMicrosecondsPrevious = 0;

    print("***StartLog\r\n");
    blackbox.start(Blackbox::start_t{.debugMode = static_cast<uint16_t>(debug.getMode()), .motorCount = 4, .servoCount = 0});

    for (size_t ii = 0; ii < 1500; ++ii) {
        const uint32_t timeMicroseconds = timeUs();

        static const uint32_t timeMicrosecondsDelta = timeMicroseconds - timeMicrosecondsPrevious;
        timeMicrosecondsPrevious = timeMicroseconds;

        const uint32_t state = blackbox.update(timeMicroseconds);
        (void)state;
        // Serial.printf("ii:%3d, s:%2d\r\n", ii, state);

        ahrs.readIMUandUpdateOrientation(timeMicroseconds, timeMicrosecondsDelta);
        receiver.update(timeMicrosecondsDelta / 1000);
#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#else
        delay(1);
#endif
    }

    //blackboxSerialDevice.endLog(true);
    blackbox.finish();
    print("***EndLog\r\n");
#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#else
    delay(5000);
#endif
}
// NOLINTEND(misc-const-correctness)

void Main::reportMainTask()
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    // The main task is set up by the framework, so just print its details.
    // It has name "loopTask" and priority 1.
    const TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    const UBaseType_t taskPriority = uxTaskPriorityGet(taskHandle);
    const char* taskName = pcTaskGetName(taskHandle);
    Serial.printf("\r\n\r\n**** Main loop task, name:'%s' priority:%u, tickRate:%uHz\r\n", taskName, taskPriority, configTICK_RATE_HZ);
#endif
}

void Main::printTaskInfo(TaskBase::task_info_t& taskInfo)
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    Serial.printf("**** %s, %.*s core:%u, priority:%u, ", taskInfo.name, 18 - strlen(taskInfo.name), "                ", taskInfo.core, taskInfo.priority);
    if (taskInfo.taskIntervalMicroseconds == 0) {
        Serial.printf("interrupt driven\r\n");
    } else {
        Serial.printf("task interval:%ums\r\n", taskInfo.taskIntervalMicroseconds / 1000);
    }
#else
    (void)taskInfo;
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
#elif defined(FRAMEWORK_STM32_CUBE)
#else
        Serial.print(&buf[0]);
#endif
}

void Main::checkGyroCalibration(NonVolatileStorage& nvs, AHRS& ahrs) // cppcheck-suppress constParameterReference
{
    // Set the gyro offsets from non-volatile storage.
    IMU_Base::xyz_int32_t offset {};
    if (nvs.GyroOffsetLoad(offset.x, offset.y, offset.z)) {
        ahrs.setGyroOffset(offset);
#if !defined(FRAMEWORK_STM32_CUBE)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** AHRS gyroOffsets loaded from NVS: gx:%5d, gy:%5d, gz:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
        print(&buf[0]);
#endif
        if (nvs.AccOffsetLoad(offset.x, offset.y, offset.z)) {
            ahrs.setAccOffset(offset);
#if !defined(FRAMEWORK_STM32_CUBE)
            sprintf(&buf[0], "**** AHRS accOffsets loaded from NVS: ax:%5d, ay:%5d, az:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
            print(&buf[0]);
#endif
        }
    } else {
        // when calibrateGyro called automatically on startup, just calibrate the gyroscope.
        //calibrateGyro(ahrs, *nvs, CALIBRATE_JUST_GYRO);
    }
}

/*!
Loads the PID profile for the FlightController. Must be called *after* the FlightController is created.
*/
void Main::loadPID_ProfileFromNonVolatileStorage(NonVolatileStorage& nvs, FlightController& flightController, uint8_t pidProfile)
{
    flightController.setFiltersConfig(nvs.FlightControllerFiltersConfigLoad(pidProfile));
    flightController.setAntiGravityConfig(nvs.FlightControllerAntiGravityConfigLoad(pidProfile));
    flightController.setDMaxConfig(nvs.FlightControllerDMaxConfigLoad(pidProfile));

    for (int ii = FlightController::PID_BEGIN; ii < FlightController::PID_COUNT; ++ii) {
        const VehicleControllerBase::PIDF_uint16_t pid = nvs.PID_load(ii, pidProfile);
        flightController.setPID_Constants(static_cast<FlightController::pid_index_e>(ii), pid);
        const std::string pidName = flightController.getPID_Name(static_cast<FlightController::pid_index_e>(ii));
#if !defined(FRAMEWORK_STM32_CUBE)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** %15s PID loaded from NVS: p:%d, i:%d, d:%d, f:%d, s:%d\r\n", pidName.c_str(), pid.kp, pid.ki, pid.kd, pid.kf, pid.ks);
        print(&buf[0]);
#else
        (void)pidName;
#endif
    }
}

/*!
The main loop handles:
1. Output to the screen
2. Input from the buttons
*/
void MainTask::loop()
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
    _tasks.mainTask->loop();
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
