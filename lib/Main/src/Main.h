#pragma once

#include <TaskBase.h>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#endif

class AHRS;
class AHRS_Task;
class Backchannel;
class BackchannelTask;
class BlackboxTask;
class ButtonsBase;
class Features;
class FlightController;
class MotorMixerBase;
class MSP_Serial;
class MSP_Task;
class RadioController;
class ReceiverBase;
class ReceiverTask;
class ReceiverWatcher;
class SV_Preferences;
class ScreenBase;
class VehicleControllerTask;

enum { MAIN_LOOP_TASK_INTERVAL_MICROSECONDS = 5000 };

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 4000 }; // 250 Hz
#endif

#if !defined(FC_TASK_INTERVAL_MICROSECONDS)
enum { FC_TASK_INTERVAL_MICROSECONDS = 4000 };
#endif

#if !defined(RECEIVER_TASK_INTERVAL_MICROSECONDS)
enum { RECEIVER_TASK_INTERVAL_MICROSECONDS = 4000 };
#endif

#if !defined(BACKCHANNEL_TASK_INTERVAL_MICROSECONDS)
enum { BACKCHANNEL_TASK_INTERVAL_MICROSECONDS = 8000 }; // 125 Hz
#endif

// MSP should run in range 100 to 2000 Hz
#if !defined(MSP_TASK_INTERVAL_MICROSECONDS)
enum { MSP_TASK_INTERVAL_MICROSECONDS = 5000 }; // 200 Hz
#endif

#if !defined(BLACKBOX_TASK_INTERVAL_MICROSECONDS)
enum { BLACKBOX_TASK_INTERVAL_MICROSECONDS = 4000 }; // 250 Hz
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
    void setup();
    void loop();
private:
    void testBlackbox(AHRS& ahrs, FlightController& flightController, RadioController& radioController, ReceiverBase& receiver);
    static AHRS& createAHRS(const MotorMixerBase& motorMixer);
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
