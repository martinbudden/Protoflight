#pragma once

#include <TaskBase.h>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#endif

class AHRS;
class AHRS_Task;
class BackchannelTask;
class BlackboxTask;
class ButtonsBase;
class Debug;
class FlightController;
class IMU_Base;
class IMU_Filters;
class IMU_FiltersBase;
class MSP_Task;
class RadioController;
class ReceiverBase;
class ReceiverTask;
class ReceiverWatcher;
class SV_Preferences;
class ScreenBase;
class VehicleControllerTask;

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
    void setup();
    void loop();
private:
    void testBlackbox(AHRS& ahrs, FlightController& flightController, const RadioController& radioController, ReceiverBase& receiver, const Debug& debug, const IMU_Filters& imuFilters);
    static IMU_Base& createIMU(uint32_t& AHRS_taskIntervalMicroSeconds);
    static AHRS& createAHRS(uint32_t AHRS_taskIntervalMicroSeconds, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters);
    static void checkGyroCalibration(SV_Preferences& preferences, AHRS& ahrs);
    static void resetPreferences(SV_Preferences& preferences, FlightController& flightController);
    static void loadPreferences(SV_Preferences& preferences, FlightController& flightController);
    static void reportMainTask();
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

    ReceiverBase* _receiver {nullptr};

    ScreenBase* _screen {nullptr};
    uint32_t _screenTickCount {0};

    ButtonsBase* _buttons {nullptr};
    uint32_t _buttonsTickCount {0};
};
