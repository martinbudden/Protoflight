#pragma once

#include "Targets.h"

#include <TaskBase.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#endif
#endif


class AHRS;
class AHRS_MessageQueue;
class AHRS_Task;
class Autopilot;
class BackchannelBase;
class BackchannelTask;
class Blackbox;
class BlackboxTask;
class ButtonsBase;
class Cockpit;
class Debug;
class FlightController;
class IMU_Base;
class IMU_Filters;
class IMU_FiltersBase;
class MSP_Task;
class MSP_SerialBase;
class NonVolatileStorage;
class OSD;
class OSD_Task;
class RPM_Filters;
class ReceiverBase;
class ReceiverTask;
class ScreenBase;
class VehicleControllerBase;
class VehicleControllerTask;

enum { MAIN_LOOP_TASK_INTERVAL_MICROSECONDS = 5000 }; // 200 Hz

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

#if !defined(OSD_TASK_INTERVAL_MICROSECONDS)
enum { OSD_TASK_INTERVAL_MICROSECONDS = 80000 }; // 12 Hz
#endif

enum {
    AHRS_TASK_PRIORITY = 6,
    FC_TASK_PRIORITY = 5,
    RECEIVER_TASK_PRIORITY = FC_TASK_PRIORITY,
    MOTORS_TASK_PRIORITY = 4,
    BACKCHANNEL_TASK_PRIORITY = 3,
    BLACKBOX_TASK_PRIORITY = 3,
    MSP_TASK_PRIORITY = 2,
    OSD_TASK_PRIORITY = 2,
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
};

class DashboardTask : public TaskBase {
public:
    explicit DashboardTask(uint32_t taskIntervalMicroseconds) : TaskBase(taskIntervalMicroseconds) {}
    void loop();
};

class Main {
public:
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7}; // Note: defining PI=8 will cause conflict with Arduino's #define of PI (3.14..)
    enum {P0=0, P1=1, P2=2, P3=3, P4=4, P5=5, P6=6, P7=7};
    enum calibration_type_e { CALIBRATE_ACC_AND_GYRO, CALIBRATE_GYRO_ONLY };
public:
    void setup();
    void loop();
private:
    static FlightController& createFlightController(uint32_t taskIntervalMicroseconds, AHRS_MessageQueue& ahrsMessageQueue, IMU_Filters& imuFilters, Debug& debug, const NonVolatileStorage& nvs);
    static IMU_Base& createIMU();
    static AHRS& createAHRS(VehicleControllerBase& vehicleController, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters);
    static ReceiverBase& createReceiver();
    static Cockpit& createCockpit(ReceiverBase& receiver, FlightController& flightController, Debug& debug, const AHRS_MessageQueue& ahrsMessageQueue, NonVolatileStorage& nvs);
    static BackchannelBase& createBackchannel(FlightController& flightController, AHRS& ahrs, ReceiverBase& receiver, const TaskBase* dashboardTask, NonVolatileStorage& nvs);
    static Blackbox& createBlackBox(AHRS& ahrs, FlightController& flightController, AHRS_MessageQueue& ahrsMessageQueue, Cockpit& cockpit, const ReceiverBase& receiver, const IMU_Filters& imuFilters, const Debug& debug);
    static OSD& createOSD(const FlightController& flightController, Debug& debug);
    static MSP_SerialBase& createMSP(AHRS& ahrs, FlightController& flightController, Cockpit& cockpit, const ReceiverBase& receiver, const Autopilot& autopilot, Debug& debug, NonVolatileStorage& nvs);

    static void testBlackbox(Blackbox& blackbox, AHRS& ahrs, ReceiverBase& receiver, const Debug& debug);

    static void checkIMU_Calibration(NonVolatileStorage& nvs, AHRS& ahrs);
    static void runIMU_Calibration(NonVolatileStorage& nvs, AHRS& ahrs, calibration_type_e calibrationType);
    static void calibrateIMU(NonVolatileStorage& nvs, AHRS& ahrs, calibration_type_e calibrationType);

    static void loadPID_ProfileFromNonVolatileStorage(FlightController& flightController, const NonVolatileStorage& nvs, uint8_t pidProfile);
    static void print(const char* buf);
    static void reportDashboardTask();
    static void printTaskInfo(TaskBase::task_info_t& taskInfo);
    struct tasks_t {
        DashboardTask* dashboardTask;
        AHRS_Task* ahrsTask;
        VehicleControllerTask* flightControllerTask;
        ReceiverTask* receiverTask;
        BackchannelTask* backchannelTask;
        MSP_Task* mspTask;
        BlackboxTask* blackboxTask;
        OSD_Task* osdTask;
    };
private:
    tasks_t _tasks {};

    ScreenBase* _screen {nullptr};
    uint32_t _screenTickCount {0};

    ButtonsBase* _buttons {nullptr};
    uint32_t _buttonsTickCount {0};
};
