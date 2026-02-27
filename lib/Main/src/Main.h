#pragma once

#include "Targets.h"
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
class IMU_Filters;
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
struct blackbox_parameter_group_t;

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
    static ImuBase& createIMU(NonVolatileStorage& nvs);
    static MotorMixerBase& createMotorMixer(float taskIntervalSeconds, const NonVolatileStorage& nvs, RpmFilters*& rpmFilters);
    static FlightController& createFlightController(float taskIntervalSeconds, const NonVolatileStorage& nvs);
    static IMU_Filters& createIMU_Filters(float taskIntervalSeconds, RpmFilters* rpmFilters, const NonVolatileStorage& nvs);
    static Ahrs& createAHRS(ImuBase& imuSensor);
    static RcModes& createRcModes(NonVolatileStorage& nvs);
    static ReceiverBase& createReceiver(NonVolatileStorage& nvs);
    static Cockpit& createCockpit(const AhrsMessageQueue& ahrsMessageQueue, NonVolatileStorage& nvs);
    static DisplayPortBase& createDisplayPort();
    // optional components: create function returns nullptr if component not specified as part of the build
    static Dashboard* createDashboard(const ReceiverBase& receiver);
    static Blackbox* createBlackBox(uint32_t task_interval_microseconds);
    static VTX* createVTX(NonVolatileStorage& nvs);
    static GPS* createGPS();
    static OSD* createOSD(DisplayPortBase& displayPort, NonVolatileStorage& nvs);
    static MspSerial* createMSP(MspBase*& msp_base);
    static CMS* createCMS();
    static BarometerBase* createBarometer();
    static BackchannelBase& createBackchannel(ReceiverBase& receiver);

    static void testBlackbox(Blackbox& blackbox, Ahrs& ahrs, ReceiverBase& receiver, FlightController& flightController, IMU_Filters& imuFilters, Debug& debug, const blackbox_parameter_group_t& pg);

    static void checkIMU_Calibration(NonVolatileStorage& nvs, ImuBase& imu);
    static void calibrateIMUandSave(NonVolatileStorage& nvs, ImuBase& imu, calibration_type_e calibrationType);

    static void load_pid_ProfileFromNonVolatileStorage(FlightController& flightController, const NonVolatileStorage& nvs, uint8_t pidProfile);
    static void print(const char* buf);
    struct tasks_t {
        DashboardTask* dashboardTask;
        AhrsTask* ahrsTask;
        MotorMixerTask* motorMixerTask;
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
