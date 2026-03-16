#include "main.h"

#include "altitude_kalman_filter.h"
#include "altitude_task.h"
#include "autopilot.h"
#include "backchannel_flight_controller.h"
#include "blackbox_callbacks.h"
#include "cms_task.h"
#include "cmsx.h"
#include "cockpit.h"
#include "dashboard.h"
#include "dashboard_task.h"
#include "flight_controller.h"
#include "imu_filters.h"
#include "msp_protoflight.h"
#include "non_volatile_storage.h"
#include "osd.h"
#include "osd_task.h"
#include "version.h"

#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <HardwareSerial.h>
#endif

#include <ahrs_message_queue.h>
#include <ahrs_task.h>
#include <backchannel_task.h>
#include <blackbox_task.h>
#include <ctime>
#include <debug.h>
#include <imu_base.h>
#include <motor_mixer_base.h>
#include <motor_mixer_message_queue.h>
#include <motor_mixer_task.h>
#include <msp_task.h>
#include <receiver_task.h>


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
    Ahrs& ahrs = create_ahrs(create_imu(nvs));
    const float ahrs_task_interval_seconds = 1.0F / static_cast<float>(ahrs.get_imu().get_gyro_sample_rate_hz());

    RpmFilters* rpm_filters = nullptr;
    MotorMixerBase& motor_mixer = create_motor_mixer(ahrs_task_interval_seconds, nvs, rpm_filters);

    FlightController& flight_controller = create_flight_controller(ahrs_task_interval_seconds, nvs);

    ImuFilters& imu_filters = create_imu_filters(ahrs_task_interval_seconds, rpm_filters, nvs); // cppcheck-suppress constVariableReference

    ReceiverBase& receiver = create_receiver(nvs);

    RcModes& rc_modes = create_rc_modes(nvs);

    Blackbox* blackbox = create_blackbox(flight_controller.get_task_interval_microseconds());

    DisplayPortBase& display_port = create_display_port();

    OSD* osd = create_osd(display_port, nvs);

    static AhrsMessageQueue ahrs_message_queue;
    Cockpit& cockpit = createCockpit(ahrs_message_queue, nvs);

    // create the optional components according to build flags
    [[maybe_unused]] VTX* vtx = create_vtx(nvs); // VTX settings may be changed by MSP or the CMS (also by CLI when it gets implemented).
    [[maybe_unused]] GPS* gps = create_gps();
    [[maybe_unused]] CMS* cms = create_cms();
    MspBase* msp_base {};
    [[maybe_unused]] MspSerial* mspSerial = create_msp(msp_base);
    [[maybe_unused]] Dashboard* dashboard = create_dashboard(receiver);

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

    struct ahrs_context_t ahrs_context = {
        .ahrs = ahrs,
        .ahrs_message_queue = ahrs_message_queue,
        .imu_filters = imu_filters,
        .vehicle_controller = flight_controller,
        .motor_mixer_message_queue = motor_mixer_message_queue,
        .debug = debug
    };
#if defined(AHRS_TASK_IS_TIMER_DRIVEN)
    const auto ahrs_task_interval_microseconds = static_cast<uint32_t>(ahrs_task_interval_seconds*1000000.0F);
    _tasks.ahrsTask = AhrsTask::create_task(task_info, ahrs_context, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, ahrs_task_interval_microseconds);
#else
    _tasks.ahrsTask = AhrsTask::create_task(task_info, ahrs_context, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, 0);
#endif

    static motor_mixer_context_t motor_mixer_context = {
        .motor_mixer_message_queue = motor_mixer_message_queue,
        .rpm_filters = rpm_filters,
        .debug = debug
    };
    _tasks.motor_mixerTask = MotorMixerTask::create_task(task_info, motor_mixer, motor_mixer_context, FC_TASK_PRIORITY, FC_TASK_CORE);
    printTaskInfo(task_info);

    static receiver_context_t receiver_context = {
        .rc_modes = rc_modes,
        .flight_controller = flight_controller,
        .motor_mixer = motor_mixer,
        .debug = debug,
        .blackbox = blackbox,
        .osd = osd
    };
    _tasks.receiverTask = ReceiverTask::create_task(task_info, receiver, cockpit, receiver_context, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_CORE, RECEIVER_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#if defined(USE_DASHBOARD)
    static dashboard_context_t dashboard_context = {
        .display_port = display_port,
        .flight_controller = flight_controller,
        .ahrs_message_queue = ahrs_message_queue,
        .motor_mixer = motor_mixer,
        .receiver = receiver
    };
    assert(dashboard != nullptr);
    _tasks.dashboardTask = DashboardTask::create_task(task_info, *dashboard, dashboard_context, DASHBOARD_TASK_PRIORITY, DASHBOARD_TASK_CORE, DASHBOARD_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
#if defined(USE_MSP)
    static msp_context_t msp_context = {
        .ahrs = ahrs,
        .flight_controller = flight_controller,
        .ahrs_message_queue = ahrs_message_queue,
        .motor_mixer = motor_mixer,
        .cockpit = cockpit,
        .receiver = receiver,
        .rc_modes = rc_modes,
        .imu_filters = imu_filters,
        .debug = debug,
        .nvs = nvs,
        .blackbox = blackbox,
        .vtx = vtx,
        .osd = osd,
        .gps = gps
    };
    _tasks.mspTask = MspTask::create_task(task_info, *mspSerial, msp_context, MSP_TASK_PRIORITY, MSP_TASK_CORE, MSP_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
#if defined(USE_BLACKBOX)
    assert(blackbox != nullptr);
    blackbox_context_t blackbox_context = {
        .ahrs_message_queue = ahrs_message_queue,
        .flight_controller = flight_controller,
        .imu_filters = imu_filters,
        .motor_mixer = motor_mixer,
        .cockpit = cockpit,
        .receiver = receiver,
        .rc_modes = rc_modes,
        .debug = debug,
        .gps = gps
    };
#if defined(USE_BLACKBOX_TEST)
    test_blackbox(blackbox, ahrs, receiver, flight_controller, imu_filters, debug, blackbox_context);
#endif
    _tasks.blackboxTask = BlackboxTask::create_task(task_info, *blackbox, ahrs_message_queue, blackbox_context, BLACKBOX_TASK_PRIORITY, BLACKBOX_TASK_CORE, BLACKBOX_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
#if defined(USE_OSD)
    const uint32_t osdTaskIntervalMicroseconds = 1'000'000 / osd->get_config().framerate_hz;
    assert(osd != nullptr);
    static osd_context_t osd_context = {
        .display_port = display_port,
        .ahrs_message_queue = ahrs_message_queue,
        .flight_controller = flight_controller,
        .cockpit = cockpit,
        .receiver = receiver,
        .rc_modes = rc_modes,
        .debug = debug,
        .vtx = vtx,
        .gps = gps
    };
    _tasks.osdTask = OSD_Task::create_task(task_info, *osd, osd_context, OSD_TASK_PRIORITY, OSD_TASK_CORE, osdTaskIntervalMicroseconds);
    printTaskInfo(task_info);
#endif
#if defined(USE_CMS)
    assert(cms != nullptr);
    static cms_context_t cms_context = {
        .display_port = display_port,
        .flight_controller = flight_controller,
        .motor_mixer = motor_mixer,
        .cockpit = cockpit,
        .imu_filters = imu_filters,
        .imu = ahrs.get_imu_mutable(),
        .rc_modes = rc_modes,
        .receiver = receiver,
        .nvs = nvs,
        .vtx = vtx
    };
    _tasks.cmsTask = CMS_Task::create_task(task_info, *cms, cms_context, CMS_TASK_PRIORITY, CMS_TASK_CORE, CMS_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
#if defined(USE_GPS) && false
    assert(gps != nullptr);
    _tasks.gpsTask = GPS_Task::create_task(task_info, *gps, GPS_TASK_PRIORITY, GPS_TASK_CORE, GPS_TASK_INTERVAL_MICROSECONDS);
    printTaskInfo(task_info);
#endif
#if defined(USE_BAROMETER) || defined(USE_GPS) || defined(USE_RANGEFINDER)
    static AltitudeKalmanFilter altitude_kalman_filter;
    BarometerBase* barometer = create_barometer();
    if (barometer) {
        assert(cockpit.get_autopilot_mutable().getAltitudeMessageQueue() != nullptr && "AltitudeMessageQueue not created");
        altitude_context_t context {
            .altitude_kalman_filter = altitude_kalman_filter,
            .ahrs_message_queue = ahrs_message_queue,
            .altitude_message_queue = *cockpit.get_autopilot_mutable().getAltitudeMessageQueue_mutable(),
            .barometer = *barometer
        };
        _tasks.altitudeTask = AltitudeTask::create_task(task_info, context, ALTITUDE_TASK_PRIORITY, ALTITUDE_TASK_CORE, ALTITUDE_TASK_INTERVAL_MICROSECONDS);
        printTaskInfo(task_info);
    }
#endif
#if defined(USE_BACKCHANNEL) && defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
    static backchannel_context_t backchannel_context = {
        .ahrs = ahrs,
        .ahrs_message_queue = ahrs_message_queue,
        .flight_controller = flight_controller,
        .motor_mixer = motor_mixer,
        .receiver = receiver,
        .main_task = _tasks.dashboardTask,
        .msp = msp_base,
#if defined(USE_MSP)
        .msp_context = &msp_context,
#else
        .msp_context = nullptr,
#endif
        .debug = debug,
        .nvs = nvs
    };

    BackchannelBase& backchannel = create_backchannel(receiver);
    _tasks.backchannelTask = BackchannelTask::create_task(task_info, backchannel, backchannel_context, BACKCHANNEL_TASK_PRIORITY, BACKCHANNEL_TASK_CORE, BACKCHANNEL_TASK_INTERVAL_MICROSECONDS);
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
