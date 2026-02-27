#pragma once

#include <task_base.h>

class AhrsMessageQueue;
class AltitudeKalmanFilter;
class AltitudeMessageQueue;
class BarometerBase;


class AltitudeTask : public TaskBase {
public:
    struct parameters_t {
        AltitudeKalmanFilter& altitudeKalmanFilter;
        AhrsMessageQueue& ahrsMessageQueue;
        AltitudeMessageQueue& altitudeMessageQueue;
        BarometerBase& barometer;
    };
public:
    AltitudeTask(uint32_t task_interval_microseconds, const parameters_t& parameters) :
        TaskBase(task_interval_microseconds),
        _altitudeKalmanFilter(parameters.altitudeKalmanFilter),
        _ahrsMessageQueue(parameters.ahrsMessageQueue),
        _altitudeMessageQueue(parameters.altitudeMessageQueue),
        _barometer(parameters.barometer)
    {}
public:
    static AltitudeTask* create_task(task_info_t& task_info, const parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static AltitudeTask* create_task(const parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
public:
    [[noreturn]] static void Task(void* arg);
private:
    [[noreturn]] void task();
private:
    AltitudeKalmanFilter& _altitudeKalmanFilter;
    AhrsMessageQueue& _ahrsMessageQueue;
    AltitudeMessageQueue& _altitudeMessageQueue;
    BarometerBase& _barometer;
};
