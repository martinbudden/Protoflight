#include "Main.h"

#include "AltitudeKalmanFilter.h"
#include "AltitudeTask.h"
#include "Autopilot.h"
#include "BackchannelFlightController.h"
#include "BlackboxCallbacks.h"
#include "CMSX.h"
#include "CMS_Task.h"
#include "Cockpit.h"
#include "Dashboard.h"
#include "DashboardTask.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "MSP_Protoflight.h"
#include "NonVolatileStorage.h"
#include "OSD.h"
#include "OSD_Task.h"
#include "version.h"

#include <backchannel_task.h>
#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <IMU_Base.h>
#include <MSP_Task.h>
#include <receiver_task.h>
//#include <VehicleControllerTask.h>

#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <HardwareSerial.h>
#endif

#include <ahrs_message_queue.h>
#include <ahrs_task.h>
#include <blackbox_task.h>
#include <ctime>
#include <debug.h>
#include <motor_mixer_message_queue.h>
#include <motor_mixer_task.h>


static void printTaskInfo(TaskBase::task_info_t& task_info);


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

#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) && !defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_ARDUINO_STM32) && !defined(FRAMEWORK_TEST)
    //Serial.begin(115200);
    //delay(500); // Allow serial port to initialize
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
    nvs.set_current_pid_profile_index(nvs.load_pid_profile_index());
    nvs.set_current_rate_profile_index(nvs.load_rate_profile_index());

    // create the AHRS and and get the IMU sample rate
    Ahrs& ahrs = createAHRS(createIMU(nvs));
    const float AHRS_taskIntervalSeconds = 1.0F / static_cast<float>(ahrs.get_imu().get_gyro_sample_rate_hz());

    RpmFilters* rpmFilters = nullptr;
    MotorMixerBase& motorMixer = createMotorMixer(AHRS_taskIntervalSeconds, nvs, rpmFilters);

    FlightController& flightController = createFlightController(AHRS_taskIntervalSeconds, nvs);

    IMU_Filters& imuFilters = createIMU_Filters(AHRS_taskIntervalSeconds, rpmFilters, nvs); // cppcheck-suppress constVariableReference

    ReceiverBase& receiver = createReceiver(nvs);

    RcModes& rc_modes = createRcModes(nvs);

    Blackbox* blackbox = createBlackBox(flightController.get_task_interval_microseconds());

    DisplayPortBase& displayPort = createDisplayPort();

    OSD* osd = createOSD(displayPort, nvs);

    static AhrsMessageQueue ahrsMessageQueue;
    Cockpit& cockpit = createCockpit(ahrsMessageQueue, nvs);

    // create the optional components according to build flags
    [[maybe_unused]] VTX* vtx = createVTX(nvs); // VTX settings may be changed by MSP or the CMS (also by CLI when it gets implemented).
    [[maybe_unused]] GPS* gps = createGPS();
    [[maybe_unused]] CMS* cms = createCMS();
    MspBase* msp_base {};
    [[maybe_unused]] MspSerial* mspSerial = createMSP(msp_base);
    [[maybe_unused]] Dashboard* dashboard = createDashboard(receiver);

#if defined(FRAMEWORK_ARDUINO_ESP32)
    //Serial.printf("\r\n\r\n%s %d.%d.%d\r\n", FC_FIRMWARE_NAME, FC_VERSION_MAJOR, FC_VERSION_MINOR, FC_VERSION_PATCH_LEVEL);
    //Serial.printf("Build Time:%d\r\n", buildTimeUnix);

    //const std::time_t timestamp = buildTimeUnix;
    //const std::tm* tm = gmtime(&timestamp);
    //Serial.printf("Build Time:%4d-%02d-%02dT%02d:%02d\r\n", tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_hour, tm->tm_min);
    //Serial.printf("Git Revision:%s\r\n\r\n\r\n", gitRevision);
#endif


    //
    // Create all the tasks
    //


    static Debug debug;
    TaskBase::task_info_t task_info {};

    static MotorMixerMessageQueue motor_mixer_message_queue;

    struct ahrs_parameter_group_t ahrs_parameter_group = {
        .ahrs = ahrs,
        .ahrs_message_queue = ahrsMessageQueue,
        .imu_filters = imuFilters,
        .vehicle_controller = flightController,
        .motor_mixer_message_queue = motor_mixer_message_queue,
        .debug = debug
    };
#if defined(AHRS_TASK_IS_TIMER_DRIVEN)
    const auto AHRS_task_interval_microseconds = static_cast<uint32_t>(AHRS_taskIntervalSeconds*1000000.0F);
    _tasks.ahrsTask = AhrsTask::create_task(task_info, ahrs_parameter_group, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, AHRS_task_interval_microseconds);
#else
    _tasks.ahrsTask = AhrsTask::create_task(task_info, ahrs_parameter_group, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, 0);
#endif
    printTaskInfo(task_info);

    static motor_mixer_parameter_group_t motor_mixer_parameter_group = {
        .motor_mixer_message_queue = motor_mixer_message_queue,
        .motor_mixer = motorMixer,
        .rpm_filters = rpmFilters,
        .debug = debug
    };
    _tasks.motorMixerTask = MotorMixerTask::create_task(task_info, motor_mixer_parameter_group, FC_TASK_PRIORITY, FC_TASK_CORE);
    printTaskInfo(task_info);

    static receiver_parameter_group_t receiver_parameter_group = {
        .rc_modes = rc_modes,
        .flight_controller = flightController,
        .motor_mixer = motorMixer,
        .debug = debug,
        .blackbox = blackbox,
        .osd = osd
    };
    _tasks.receiverTask = ReceiverTask::create_task(task_info, receiver, cockpit, receiver_parameter_group, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_CORE, RECEIVER_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#if defined(USE_DASHBOARD)
    static dashboard_parameter_group_t dashboard_parameter_group = {
        .displayPort = displayPort,
        .flightController = flightController,
        .ahrsMessageQueue = ahrsMessageQueue,
        .motorMixer = motorMixer,
        .receiver = receiver
    };
    assert(dashboard != nullptr);
    _tasks.dashboardTask = DashboardTask::create_task(task_info, *dashboard, dashboard_parameter_group, DASHBOARD_TASK_PRIORITY, DASHBOARD_TASK_CORE, DASHBOARD_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
#if defined(USE_MSP)
    static msp_parameter_group_t msp_parameter_group = {
        .ahrs = ahrs,
        .flightController = flightController,
        .ahrsMessageQueue = ahrsMessageQueue,
        .motorMixer = motorMixer,
        .cockpit = cockpit,
        .receiver = receiver,
        .rc_modes = rc_modes,
        .imuFilters = imuFilters,
        .debug = debug,
        .nonVolatileStorage = nvs,
        .blackbox = blackbox,
        .vtx = vtx,
        .osd = osd,
        .gps = gps
    };
    _tasks.mspTask = MspTask::create_task(task_info, *mspSerial, msp_parameter_group, MSP_TASK_PRIORITY, MSP_TASK_CORE, MSP_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
#if defined(USE_BLACKBOX)
    assert(blackbox != nullptr);
    blackbox_parameter_group_t blackbox_parameter_group = {
        .ahrs_message_queue = ahrsMessageQueue,
        .flightController = flightController,
        .imuFilters = imuFilters,
        .motorMixer = motorMixer,
        .cockpit = cockpit,
        .receiver = receiver,
        .rc_modes = rc_modes,
        .debug = debug,
        .gps = gps
    };
#if defined(USE_BLACKBOX_TEST)
    testBlackbox(blackbox, ahrs, receiver, flightController, imuFilters, debug, blackbox_parameter_group);
#endif
    _tasks.blackboxTask = BlackboxTask::create_task(task_info, *blackbox, ahrsMessageQueue, blackbox_parameter_group, BLACKBOX_TASK_PRIORITY, BLACKBOX_TASK_CORE, BLACKBOX_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
#if defined(USE_OSD)
    const uint32_t osdTaskIntervalMicroseconds = 1'000'000 / osd->getConfig().framerate_hz;
    assert(osd != nullptr);
    static osd_parameter_group_t osd_parameter_group = {
        .displayPort = displayPort,
        .ahrs_message_queue = ahrsMessageQueue,
        .flightController = flightController,
        .cockpit = cockpit,
        .receiver = receiver,
        .rc_modes = rc_modes,
        .debug = debug,
        .vtx = vtx,
        .gps = gps
    };
    _tasks.osdTask = OSD_Task::create_task(task_info, *osd, osd_parameter_group, OSD_TASK_PRIORITY, OSD_TASK_CORE, osdTaskIntervalMicroseconds);
    printTaskInfo(task_info);
#endif
#if defined(USE_CMS)
    assert(cms != nullptr);
    static cms_parameter_group_t cms_parameter_group = {
        .displayPort = displayPort,
        .flightController = flightController,
        .motorMixer = motorMixer,
        .cockpit = cockpit,
        .imuFilters = imuFilters,
        .imu = ahrs.get_imu_mutable(),
        .rc_modes = rc_modes,
        .receiver = receiver,
        .nvs = nvs,
        .vtx = vtx
    };
    _tasks.cmsTask = CMS_Task::create_task(task_info, *cms, cms_parameter_group, CMS_TASK_PRIORITY, CMS_TASK_CORE, CMS_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
#if defined(USE_GPS) && false
    assert(gps != nullptr);
    _tasks.gpsTask = GPS_Task::create_task(task_info, *gps, GPS_TASK_PRIORITY, GPS_TASK_CORE, GPS_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
#if defined(USE_BAROMETER) || defined(USE_GPS) || defined(USE_RANGEFINDER)
    static AltitudeKalmanFilter altitudeKalmanFilter;
    BarometerBase* barometer = createBarometer();
    if (barometer) {
        assert(cockpit.getAutopilotMutable().getAltitudeMessageQueue() != nullptr && "AltitudeMessageQueue not created");
        const AltitudeTask::parameters_t parameters {
            .altitudeKalmanFilter = altitudeKalmanFilter,
            .ahrsMessageQueue = ahrsMessageQueue,
            .altitudeMessageQueue = *cockpit.getAutopilotMutable().getAltitudeMessageQueueMutable(),
            .barometer = *barometer
        };
        _tasks.altitudeTask = AltitudeTask::create_task(task_info, parameters, ALTITUDE_TASK_PRIORITY, ALTITUDE_TASK_CORE, ALTITUDE_TASK_INTERVAL_MICROSECONDS);
        printTaskInfo(task_info);
    }
#endif
#if defined(USE_BACKCHANNEL) && defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
    static backchannel_parameter_group_t backchannel_parameter_group = {
        .ahrs = ahrs,
        .ahrs_message_queue = ahrsMessageQueue,
        .flight_controller = flightController,
        .motor_mixer = motorMixer,
        .receiver = receiver,
        .main_task = _tasks.dashboardTask,
        .msp = msp_base,
#if defined(USE_MSP)
        .msp_parameter_group = &msp_parameter_group,
#else
        .msp_parameter_group = nullptr,
#endif
        .debug = debug,
        .nvs = nvs
    };

    BackchannelBase& backchannel = createBackchannel(receiver);
    _tasks.backchannelTask = BackchannelTask::create_task(task_info, backchannel, backchannel_parameter_group, BACKCHANNEL_TASK_PRIORITY, BACKCHANNEL_TASK_CORE, BACKCHANNEL_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
}


void printTaskInfo(TaskBase::task_info_t& task_info)
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    Serial.printf("**** %s, %.*s core:%d, priority:%d, ", task_info.name, 18 - strlen(task_info.name), "                ", static_cast<int>(task_info.core), static_cast<int>(task_info.priority));
    if (task_info.task_interval_microseconds == 0) {
        Serial.printf("interrupt driven\r\n");
    } else {
        if (task_info.task_interval_microseconds / 1000 == 0) {
            Serial.printf("task interval:%4uus\r\n", static_cast<unsigned int>(task_info.task_interval_microseconds));
        } else {
            Serial.printf("task interval:%3ums\r\n", static_cast<unsigned int>(task_info.task_interval_microseconds / 1000));
        }
    }
#else
    std::array<char, 128> buf;
    sprintf(&buf[0], "**** %s, %.*s core:%d, priority:%d, ", task_info.name, static_cast<int>(18 - strlen(task_info.name)), "                ", static_cast<int>(task_info.core), static_cast<int>(task_info.priority));
    printf(&buf[0]);
    if (task_info.task_interval_microseconds == 0) {
        printf("interrupt driven\r\n");
    } else {
        sprintf(&buf[0], "task interval:%ums\r\n", static_cast<unsigned int>(task_info.task_interval_microseconds / 1000));
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
#if defined(FRAMEWORK_ARDUINO_ESP32)
    Serial.print(&buf[0]);
#else
    (void)buf;
#endif
#endif
}
