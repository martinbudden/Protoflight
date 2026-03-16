#pragma once

#include "targets.h"
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <soc/soc.h>
#endif

#include <cstdint>


class Ahrs;
class AhrsMessageQueue;
class AhrsTask;
class AltitudeMessageQueue;
class AltitudeTask;
class Autopilot;
class BackchannelBase;
class BackchannelTask;
class BarometerBase;
class Blackbox;
class BlackboxTask;
class ButtonsBase;
class CMS;
class CMS_Task;
class Cockpit;
class Dashboard;
class DashboardTask;
class Debug;
class DisplayPortBase;
class FlightController;
class GPS;
class GPS_Task;
class ImuBase;
class ImuFilters;
class ImuFiltersBase;
class MspBase;
class MspTask;
class MspSerial;
class MotorMixerBase;
class MotorMixerTask;
class MotorMixerMessageQueue;
class NonVolatileStorage;
class OSD;
class OSD_Task;
class RpmFilters;
class RcModes;
class ReceiverBase;
class ReceiverTask;
class ScreenBase;
class TaskBase;
class VTX;
class VehicleControllerBase;
struct blackbox_context_t;

#if !defined(DASHBOARD_TASK_INTERVAL_MICROSECONDS)
enum { DASHBOARD_TASK_INTERVAL_MICROSECONDS = 40000 }; // 25 Hz
#endif

#if !defined(GYRO_SAMPLE_RATE_HZ)
enum { GYRO_SAMPLE_RATE_HZ = 2000 }; // 2000 Hz, 500 microseconds looptime
#endif

#if !defined(OUTPUT_TO_MOTORS_DENOMINATOR)
enum { OUTPUT_TO_MOTORS_DENOMINATOR = 2 }; // runs at half rate of AHRS_TASK
#endif

#if !defined(RECEIVER_TASK_INTERVAL_MICROSECONDS)
enum { RECEIVER_TASK_INTERVAL_MICROSECONDS = 4000 }; // 250 Hz
#endif

#if !defined(BACKCHANNEL_TASK_INTERVAL_MICROSECONDS)
enum { BACKCHANNEL_TASK_INTERVAL_MICROSECONDS = 8000 }; // 125 Hz
#endif

// MSP should run in range 100 to 2000 Hz
#if !defined(MSP_TASK_INTERVAL_MICROSECONDS)
enum { MSP_TASK_INTERVAL_MICROSECONDS = 5000 }; // 200 Hz
#endif

#if !defined(BLACKBOX_TASK_INTERVAL_MICROSECONDS)
enum { BLACKBOX_TASK_INTERVAL_MICROSECONDS = 2000 }; // 500 Hz
#endif

#if !defined(CMS_TASK_INTERVAL_MICROSECONDS)
enum { CMS_TASK_INTERVAL_MICROSECONDS = 50000 }; // 20 Hz
#endif

#if !defined(GPS_TASK_INTERVAL_MICROSECONDS)
enum { GPS_TASK_INTERVAL_MICROSECONDS = 4000 }; // 250 Hz
#endif

#if !defined(ALTITUDE_TASK_INTERVAL_MICROSECONDS)
enum { ALTITUDE_TASK_INTERVAL_MICROSECONDS = 2000 }; // 500 Hz
#endif

enum {
    AHRS_TASK_PRIORITY = 6,
    FC_TASK_PRIORITY = 5,
    RECEIVER_TASK_PRIORITY = FC_TASK_PRIORITY,
    MOTORS_TASK_PRIORITY = 4,
    BACKCHANNEL_TASK_PRIORITY = RECEIVER_TASK_PRIORITY, // to avoid priority inversion between receiver task and backchannel
    BLACKBOX_TASK_PRIORITY = 3,
    MSP_TASK_PRIORITY = 2,
    OSD_TASK_PRIORITY = 2,
    CMS_TASK_PRIORITY = 2,
    GPS_TASK_PRIORITY = 2,
    DASHBOARD_TASK_PRIORITY = 2,
    ALTITUDE_TASK_PRIORITY = 2,
};

#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)

enum  { CPU_CORE_0 = PRO_CPU_NUM };
#if defined(APP_CPU_NUM)
enum  { CPU_CORE_1 = APP_CPU_NUM };
#else
enum  { CPU_CORE_1 = CPU_CORE_0 }; // the processor has only one core
#endif

#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)

enum  { CPU_CORE_0 = 0x01, CPU_CORE_1 = 0x02 }; // bitmask is used for core affinity for RPI_PICO

#else

enum  { CPU_CORE_0 = 0, CPU_CORE_1 = 0 };

#endif // FRAMEWORK

enum {
    AHRS_TASK_CORE = CPU_CORE_1, // AHRS should be the only task running on core 1
    FC_TASK_CORE = CPU_CORE_0,
    RECEIVER_TASK_CORE = CPU_CORE_0,
    BACKCHANNEL_TASK_CORE = CPU_CORE_0,
    MOTORS_TASK_CORE = CPU_CORE_0,
    MSP_TASK_CORE = CPU_CORE_0,
    BLACKBOX_TASK_CORE = CPU_CORE_0,
    OSD_TASK_CORE = CPU_CORE_0,
    CMS_TASK_CORE = CPU_CORE_0,
    GPS_TASK_CORE = CPU_CORE_0,
    DASHBOARD_TASK_CORE = CPU_CORE_0,
    ALTITUDE_TASK_CORE = CPU_CORE_0,
};

class Main {
public:
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7}; // Note: defining PI=8 will cause conflict with Arduino's #define of PI (3.14..)
    enum {P0=0, P1=1, P2=2, P3=3, P4=4, P5=5, P6=6, P7=7};
    enum calibration_type_e { CALIBRATE_ACC_AND_GYRO, CALIBRATE_GYRO_ONLY };
public:
    void setup();
private:
    static ImuBase& create_imu(NonVolatileStorage& nvs);
    static MotorMixerBase& create_motor_mixer(float task_interval_seconds, const NonVolatileStorage& nvs, RpmFilters*& rpm_filters_ptr);
    static FlightController& create_flight_controller(float task_interval_seconds, const NonVolatileStorage& nvs);
    static ImuFilters& create_imu_filters(float task_interval_seconds, RpmFilters* rpm_filters, const NonVolatileStorage& nvs);
    static Ahrs& create_ahrs(ImuBase& imuSensor);
    static RcModes& create_rc_modes(NonVolatileStorage& nvs);
    static ReceiverBase& create_receiver(NonVolatileStorage& nvs);
    static Cockpit& createCockpit(const AhrsMessageQueue& ahrs_message_queue, NonVolatileStorage& nvs);
    static DisplayPortBase& create_display_port();
    // optional components: create function returns nullptr if component not specified as part of the build
    static Dashboard* create_dashboard(const ReceiverBase& receiver);
    static Blackbox* create_blackbox(uint32_t task_interval_microseconds);
    static VTX* create_vtx(NonVolatileStorage& nvs);
    static GPS* create_gps();
    static OSD* create_osd(DisplayPortBase& display_port, NonVolatileStorage& nvs);
    static MspSerial* create_msp(MspBase*& msp_base_ptr);
    static CMS* create_cms();
    static BarometerBase* create_barometer();
    static BackchannelBase& create_backchannel(ReceiverBase& receiver);

    static void test_blackbox(Blackbox& blackbox, Ahrs& ahrs, ReceiverBase& receiver, FlightController& flight_controller, ImuFilters& imu_filters, Debug& debug, const blackbox_context_t& ctx);

    static void check_imu_calibration(NonVolatileStorage& nvs, ImuBase& imu);
    static void calibrate_imu_and_save(NonVolatileStorage& nvs, ImuBase& imu, calibration_type_e calibration_type);

    static void load_pid_ProfileFromNonVolatileStorage(FlightController& flight_controller, const NonVolatileStorage& nvs, uint8_t pid_profile);
    static void print(const char* buf);
    struct tasks_t {
        DashboardTask* dashboardTask;
        AhrsTask* ahrsTask;
        MotorMixerTask* motor_mixerTask;
        ReceiverTask* receiverTask;
        BackchannelTask* backchannelTask;
        MspTask* mspTask;
        BlackboxTask* blackboxTask;
        OSD_Task* osdTask;
        CMS_Task* cmsTask;
        GPS_Task* gpsTask;
        AltitudeTask* altitudeTask;
    };
private:
    tasks_t _tasks {};
};
