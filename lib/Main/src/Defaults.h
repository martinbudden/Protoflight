#pragma once

#include <MotorMixerBase.h>


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


namespace DEFAULTS {

const MotorMixerBase::dynamic_idle_controller_config_t dynamicIdleControllerConfig = {
    .minRPM = 0,
    .maxIncrease = 150,
    .kp = 50,
    .ki = 50,
    .kd = 50,
};

} // END namespace