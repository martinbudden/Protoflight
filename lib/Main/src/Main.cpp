#include "Main.h"

#if defined(M5_UNIFIED)
#include "ButtonsM5.h"
#include "ScreenM5.h"
#endif

#include <AHRS.h>
#include <AHRS_Task.h>
#include <BackchannelFlightController.h>
#include <BackchannelTask.h>
#if defined(USE_ESPNOW)
#include <BackchannelTransceiverESPNOW.h>
#endif
#include <BlackboxCallbacks.h>
#include <BlackboxMessageQueueAHRS.h>
#include <BlackboxProtoFlight.h>
#include <BlackboxSerialDeviceSDCard.h>
#include <BlackboxTask.h>
#include <Features.h>
#include <FlightController.h>
#include <MSP_ProtoFlight.h>
#include <MSP_Serial.h>
#include <MSP_Task.h>
#include <MotorMixerQuadX_DShot.h>
#include <MotorMixerQuadX_PWM.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <RadioController.h>
#include <ReceiverAtomJoyStick.h>
#include <ReceiverNull.h>
#include <ReceiverTask.h>
#include <SV_Preferences.h>
#include <TimeMicroSeconds.h>
#include <VehicleControllerTask.h>
#if defined(USE_ESPNOW)
#include <WiFi.h>
#endif

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#endif


/*!
Setup for the main loop, motor control task, and AHRS(Attitude and Heading Reference System) task.
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

#if !defined(FRAMEWORK_RPI_PICO)
    Serial.begin(115200);
#endif

#if defined(USE_ESPNOW)
    // Set WiFi to station mode
    WiFi.mode(WIFI_STA);
    // Disconnect from Access Point if it was previously connected
    WiFi.disconnect();
    // get my MAC address
    uint8_t myMacAddress[ESP_NOW_ETH_ALEN];
    WiFi.macAddress(&myMacAddress[0]);

    // Statically allocate and setup the receiver.
    static ReceiverAtomJoyStick receiver(&myMacAddress[0]);
    _receiver = &receiver;
    static RadioController radioController(receiver);
#if !defined(RECEIVER_CHANNEL)
    enum { RECEIVER_CHANNEL = 3 };
#endif
    const esp_err_t espErr = receiver.setup(RECEIVER_CHANNEL);
    Serial.printf("\r\n\r\n**** ESP-NOW Ready:%X\r\n\r\n", espErr);
    assert(espErr == ESP_OK && "Unable to setup receiver.");
#else
    // Statically allocate and setup the receiver.
    static ReceiverNull receiver;
    _receiver = &receiver;
    static RadioController radioController(receiver);
#endif // USE_ESPNOW

    // Statically allocate the MotorMixer object as defined by the build flags.
    const float deltaT = static_cast<float>(FC_TASK_INTERVAL_MICROSECONDS) * 0.000001F;
#if defined(USE_MOTOR_MIXER_QUAD_X_PWM)
    const MotorMixerQuadX_PWM::pins_t pins = MOTOR_PINS;
    static MotorMixerQuadX_PWM motorMixer(pins, deltaT); // NOLINT(misc-const-correctness) false positive
#elif defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)
    const MotorMixerQuadX_DShot::pins_t pins = MOTOR_PINS;
    static MotorMixerQuadX_DShot motorMixer(pins, deltaT); // NOLINT(misc-const-correctness) false positive
#endif

    // Statically allocate the AHRS
    AHRS& ahrs = createAHRS(motorMixer);

    // Statically allocate the flightController.
    static FlightController flightController(FC_TASK_INTERVAL_MICROSECONDS, ahrs, motorMixer, radioController);
    ahrs.setVehicleController(&flightController);
    radioController.setFlightController(&flightController);

    // Statically allocate the MSP and associated objects
#define USE_MSP
#if defined(USE_MSP)
    static Features features; // NOLINT(misc-const-correctness) false positive
    static MSP_ProtoFlight mspProtoFlight(features, ahrs, flightController, radioController, receiver); // NOLINT(misc-const-correctness) false positive
    static MSP_Stream mspStream(mspProtoFlight);
    static MSP_Serial mspSerial(mspStream); // NOLINT(misc-const-correctness) false positive
#endif

    // Statically allocate the Blackbox and associated objects
//#define USE_BLACKBOX
#if !defined(USE_BLACKBOX) && defined(USE_BLACKBOX_DEBUG)
    testBlackbox(ahrs, flightController, radioController, receiver);
#endif
#if defined(USE_BLACKBOX)
    static BlackboxMessageQueue blackboxMessageQueue;
    static BlackboxCallbacks blackboxCallbacks(blackboxMessageQueue, ahrs, flightController, radioController, receiver);
    static BlackboxSerialDeviceSDCard blackboxSerialDevice;
    blackboxSerialDevice.init();
    static BlackboxProtoFlight blackbox(blackboxCallbacks, blackboxMessageQueue, blackboxSerialDevice, flightController, radioController);
    static BlackboxMessageQueueAHRS blackboxMessageQueueAHRS(blackboxMessageQueue);
    ahrs.setMessageQueue(&blackboxMessageQueueAHRS);
    flightController.setBlackbox(blackbox);
    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        //.device = Blackbox::DEVICE_NONE,
        .mode = Blackbox::MODE_NORMAL // logging starts on arming, file is saved when disarmed
        //.mode = Blackbox::MODE_ALWAYS_ON
    });
#endif
#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(USE_ESPNOW)
    static SV_Preferences preferences;
#endif

#if defined(M5_UNIFIED)
    // Holding BtnA down while switching on enters calibration mode.
    //if (M5.BtnA.isPressed()) {
    //    calibrateGyro(*ahrs, preferences, CALIBRATE_ACC_AND_GYRO);
    //}
#endif
    //!!checkGyroCalibration(preferences, ahrs);

#if defined(M5_UNIFIED)
    // Holding BtnC down while switching on resets the preferences.
    if (M5.BtnC.isPressed()) {
        resetPreferences(preferences, flightController);
    }
#endif
    //!!loadPreferences(preferences, flightController);

#if defined(M5_UNIFIED)
    // Statically allocate the screen.
    static ScreenM5 screen(ahrs, flightController, receiver);
    ReceiverWatcher* receiverWatcher = &screen;
    _screen = &screen;
    _screen->updateTemplate(); // Update the screen as soon as we can, to minimize the time the screen is blank

    static ReceiverTask receiverTask(RECEIVER_TASK_INTERVAL_MICROSECONDS, receiver, radioController, &screen);
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
    _tasks.ahrsTask = AHRS_Task::createTask(ahrs, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, AHRS_TASK_INTERVAL_MICROSECONDS);
    _tasks.flightControllerTask = VehicleControllerTask::createTask(flightController, FC_TASK_PRIORITY, FC_TASK_CORE);
    _tasks.receiverTask = ReceiverTask::createTask(receiver, radioController, receiverWatcher, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_CORE, RECEIVER_TASK_INTERVAL_MICROSECONDS);
#if defined(USE_MSP)
    _tasks.mspTask = MSP_Task::createTask(mspSerial, MSP_TASK_PRIORITY, MSP_TASK_CORE, MSP_TASK_INTERVAL_MICROSECONDS);
#endif
#if defined(USE_BLACKBOX)
    TaskBase::task_info_t taskInfo {}; // NOLINT(misc-const-correctness) false positive
    _tasks.blackboxTask = BlackboxTask::createTask(taskInfo, blackbox, BLACKBOX_TASK_PRIORITY, BLACKBOX_TASK_CORE, BLACKBOX_TASK_INTERVAL_MICROSECONDS);
    vTaskResume(taskInfo.taskHandle);
#endif

#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(USE_ESPNOW)
    // statically allocate an MSP object
    // static MSP_ProtoFlight mspProtoFlightBackchannel(features, ahrs, flightController, radioController, receiver); // NOLINT(misc-const-correctness) false positive
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
        preferences
    );

    _tasks.backchannelTask = BackchannelTask::createTask(backchannel, BACKCHANNEL_TASK_PRIORITY, BACKCHANNEL_TASK_CORE, BACKCHANNEL_TASK_INTERVAL_MICROSECONDS);
#endif
}

void Main::testBlackbox(AHRS& ahrs, FlightController& flightController, RadioController& radioController, ReceiverBase& receiver)
{
    static BlackboxMessageQueue blackboxMessageQueue; // NOLINT(misc-const-correctness) false positive
    static BlackboxCallbacks blackboxCallbacks(blackboxMessageQueue, ahrs, flightController, radioController, receiver); // NOLINT(misc-const-correctness) false positive
    static BlackboxSerialDeviceSDCard blackboxSerialDevice;
    blackboxSerialDevice.init();

    static BlackboxProtoFlight blackbox(blackboxCallbacks, blackboxMessageQueue, blackboxSerialDevice, flightController, radioController);
    flightController.setBlackbox(blackbox);
    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        //.device = Blackbox::DEVICE_NONE,
        .mode = Blackbox::MODE_NORMAL // logging starts on arming, file is saved when disarmed
        //.mode = Blackbox::MODE_ALWAYS_ON
    });

    static uint32_t timeMicroSecondsPrevious = 0;

#if defined(FRAMEWORK_RPI_PICO)
    printf("***StartLog\r\n");
#else
    Serial.printf("***StartLog\r\n");
#endif
    enum { DEBUG_MODE_RX_STATE_TIME = 76 };
    blackbox.start({
        .debugMode = DEBUG_MODE_RX_STATE_TIME,
        .motorCount = 4,
        .servoCount = 0
    });


    for (size_t ii = 0; ii < 1500; ++ii) {
        const uint32_t timeMicroSeconds = timeUs();

        static const uint32_t timeMicroSecondsDelta = timeMicroSeconds - timeMicroSecondsPrevious;
        timeMicroSecondsPrevious = timeMicroSeconds;

        const int state = blackbox.update(timeMicroSeconds);
        (void)state;
        // Serial.printf("ii:%3d, s:%2d\r\n", ii, state);

        ahrs.readIMUandUpdateOrientation(timeMicroSeconds, timeMicroSecondsDelta);
        receiver.update(timeMicroSecondsDelta / 1000);
#if defined(FRAMEWORK_RPI_PICO)
#else
        delay(1);
#endif
    }

    //blackboxSerialDevice.endLog(true);
    blackbox.finish();
#if defined(FRAMEWORK_RPI_PICO)
    printf("***EndLog\r\n");
#else
    Serial.printf("***EndLog\r\n");
    delay(5000);
#endif
}

void Main::reportMainTask()
{
#if defined(USE_ARDUINO_ESP32)
    // The main task is set up by the framework, so just print its details.
    // It has name "loopTask" and priority 1.
    const TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    const UBaseType_t taskPriority = uxTaskPriorityGet(taskHandle);
    const char* taskName = pcTaskGetName(taskHandle);
    Serial.printf("\r\n\r\n**** Main loop task, name:'%s' priority:%u, tickRate:%uHz\r\n", taskName, taskPriority, configTICK_RATE_HZ);
#endif
}

#if defined(USE_FREERTOS)
[[noreturn]] void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    assert(false && "stack overflow");
    Serial.printf("\r\n\r\n*********\r\n");
    Serial.printf("********Task '%s' stack overflow ********\r\n", pcTaskName);
    Serial.printf("*********\r\n\r\n");
}
#endif

void Main::checkGyroCalibration(SV_Preferences& preferences, AHRS& ahrs)
{
    // Set the gyro offsets from non-volatile storage.
    IMU_Base::xyz_int32_t offset {};
    if (preferences.getGyroOffset(offset.x, offset.y, offset.z)) {
        ahrs.setGyroOffset(offset);
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** AHRS gyroOffsets loaded from preferences: gx:%5d, gy:%5d, gz:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
#if defined(FRAMEWORK_RPI_PICO)
        printf(&buf[0]);
#else
        Serial.print(&buf[0]);
#endif
        if (preferences.getAccOffset(offset.x, offset.y, offset.z)) {
            ahrs.setAccOffset(offset);
            sprintf(&buf[0], "**** AHRS accOffsets loaded from preferences: ax:%5d, ay:%5d, az:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
#if defined(FRAMEWORK_RPI_PICO)
            printf(&buf[0]);
#else
            Serial.print(&buf[0]);
#endif
        }
    } else {
        // when calibrateGyro called automatically on startup, just calibrate the gyroscope.
        //calibrateGyro(ahrs, *preferences, CALIBRATE_JUST_GYRO);
    }
}

/*!
Resets the PID preferences to SV_Preferences::NOT_SET (which represents unset).
*/
void Main::resetPreferences(SV_Preferences& preferences, FlightController& flightController)
{
    //preferences.clear();
    //preferences.removeGyroOffset();
    //preferences.removeAccOffset();
    for (int ii = FlightController::PID_BEGIN; ii < FlightController::PID_COUNT; ++ii) {
        const std::string pidName = flightController.getPID_Name(static_cast<FlightController::pid_index_e>(ii));
        constexpr PIDF::PIDF_t pidNOT_SET { SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET };
        preferences.putPID(pidName, pidNOT_SET);
    }
#if defined(FRAMEWORK_RPI_PICO)
        printf("**** preferences reset\r\n");
#else
        Serial.println("**** preferences reset");
#endif
}

/*!
Loads the PID settings for the FlightController. Must be called *after* the FlightController is created.
*/
void Main::loadPreferences(SV_Preferences& preferences, FlightController& flightController)
{
    // Set all the preferences to zero if they have not been set
    if (!preferences.isSetPID()) {
        resetPreferences(preferences, flightController);
    }
    // Load the PID constants from preferences, and if they are non-zero then use them to set the FlightController PIDs.
    for (int ii = FlightController::PID_BEGIN; ii < FlightController::PID_COUNT; ++ii) {
        std::string pidName = flightController.getPID_Name(static_cast<FlightController::pid_index_e>(ii));
        const PIDF::PIDF_t pid = preferences.getPID(pidName);
        if (pid.kp != SV_Preferences::NOT_SET) {
            flightController.setPID_Constants(static_cast<FlightController::pid_index_e>(ii), pid);
            std::array<char, 128> buf;
            sprintf(&buf[0], "**** %s PID loaded from preferences: P:%f, I:%f, D:%f, F:%f\r\n", pidName.c_str(), static_cast<double>(pid.kp), static_cast<double>(pid.ki), static_cast<double>(pid.kd), static_cast<double>(pid.kf));
#if defined(FRAMEWORK_RPI_PICO)
            printf(&buf[0]);
#else
            Serial.print(&buf[0]);
#endif
        }
    }
}

/*!
The main loop handles:
1. Input from the receiver(joystick)
2. Input from the backchannel(PID tuning).
3. Input from the buttons
4. Output to the backchannel(telemetry).
5. Output to the screen

The IMU(Inertial Measurement Unit) is read in the AHRS(Attitude and Heading Reference System) task.
The motors are controlled in the FlightController task.
*/
void MainTask::loop()
{
#if defined(USE_FREERTOS)
    const TickType_t tickCount = xTaskGetTickCount();
    _tickCountDelta = tickCount - _tickCountPrevious;
    _tickCountPrevious = tickCount;
#endif // USE_FREERTOS
}

void Main::loop() // NOLINT(readability-make-member-function-const)
{
#if defined(USE_FREERTOS)
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
