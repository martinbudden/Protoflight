#pragma once

#include "Targets.h"

#include <TaskBase.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#endif

class AHRS;
class AHRS_Task;
class BackchannelTask;
class Blackbox;
class BlackboxTask;
class ButtonsBase;
class Debug;
class FlightController;
class IMU_Base;
class IMU_Filters;
class IMU_FiltersBase;
class MSP_Task;
class NonVolatileStorage;
class RadioController;
class ReceiverBase;
class ReceiverTask;
class ReceiverWatcher;
class ScreenBase;
class VehicleControllerTask;

enum { MAIN_LOOP_TASK_INTERVAL_MICROSECONDS = 5000 }; // 200 Hz

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 1000 }; // 1000 Hz
#endif

#if !defined(FC_TASK_DENOMINATOR)
enum { FC_TASK_DENOMINATOR = 2 }; // runs at half rate of AHRS_TASK
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

enum {
    AHRS_TASK_PRIORITY = 6,
    FC_TASK_PRIORITY = 5,
    RECEIVER_TASK_PRIORITY = FC_TASK_PRIORITY,
    MOTORS_TASK_PRIORITY = 4,
    BACKCHANNEL_TASK_PRIORITY = 3,
    MSP_TASK_PRIORITY = 2,
    BLACKBOX_TASK_PRIORITY = 3
};

#if !defined(PRO_CPU_NUM)
#define PRO_CPU_NUM (0)
#endif
#if !defined(APP_CPU_NUM)
// the processor has only one core
#define APP_CPU_NUM PRO_CPU_NUM
#endif

enum {
    AHRS_TASK_CORE = APP_CPU_NUM, // AHRS should be the only task running on the second core
    FC_TASK_CORE = PRO_CPU_NUM,
    RECEIVER_TASK_CORE = PRO_CPU_NUM,
    BACKCHANNEL_TASK_CORE = PRO_CPU_NUM,
    MOTORS_TASK_CORE = PRO_CPU_NUM,
    MSP_TASK_CORE = PRO_CPU_NUM,
    BLACKBOX_TASK_CORE = PRO_CPU_NUM,
};

class MainTask : public TaskBase {
public:
    explicit MainTask(uint32_t taskIntervalMicroSeconds) : TaskBase(taskIntervalMicroSeconds) {}
    void loop();
};

class Main {
public:
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7}; // Note: defining PI=8 will cause conflict with Arduino's #define of PI (3.14..)
    enum {P0=0, P1=1, P2=2, P3=3, P4=4, P5=5, P6=6, P7=7};
public:
    void setup();
    void loop();
private:
    void testBlackbox(Blackbox& blackbox, AHRS& ahrs, ReceiverBase& receiver, const Debug& debug);
    static IMU_Base& createIMU(int32_t& imuSampleRateHz);
    static AHRS& createAHRS(uint32_t AHRS_taskIntervalMicroSeconds, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters);
    static void checkGyroCalibration(NonVolatileStorage& nvs, AHRS& ahrs);
    static void setPIDsFromNonVolatileStorage(NonVolatileStorage& nvs, FlightController& flightController);
    static void reportMainTask();
    static void printTaskInfo(TaskBase::task_info_t& taskInfo);
    struct tasks_t {
        MainTask* mainTask;
        AHRS_Task* ahrsTask;
        VehicleControllerTask* flightControllerTask;
        ReceiverTask* receiverTask;
        BackchannelTask* backchannelTask;
        MSP_Task* mspTask;
        BlackboxTask* blackboxTask;
    };
private:
    tasks_t _tasks {};

    ScreenBase* _screen {nullptr};
    uint32_t _screenTickCount {0};

    ButtonsBase* _buttons {nullptr};
    uint32_t _buttonsTickCount {0};
};
