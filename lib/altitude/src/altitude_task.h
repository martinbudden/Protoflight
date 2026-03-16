#pragma once

#include <task_base.h>

class AhrsMessageQueue;
class AltitudeKalmanFilter;
class AltitudeMessageQueue;
class BarometerBase;


struct altitude_context_t {
    AltitudeKalmanFilter& altitude_kalman_filter;
    AhrsMessageQueue& ahrs_message_queue;
    AltitudeMessageQueue& altitude_message_queue;
    BarometerBase& barometer;
};

class AltitudeTask : public TaskBase {
public:
public:
    AltitudeTask(uint32_t task_interval_microseconds, altitude_context_t& context) :
        TaskBase(task_interval_microseconds),
        _context(context)
    {}
public:
    static AltitudeTask* create_task(task_info_t& task_info, altitude_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static AltitudeTask* create_task(altitude_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
public:
    [[noreturn]] static void task_static(void* arg);
private:
    [[noreturn]] void task();
private:
    altitude_context_t& _context;
};
