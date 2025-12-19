#pragma once

#include <TaskBase.h>

class AHRS_MessageQueue;
class AltitudeKalmanFilter;
class AltitudeMessageQueue;
class BarometerBase;


class AltitudeTask : public TaskBase {
public:
    struct parameters_t {
        AltitudeKalmanFilter& altitudeKalmanFilter;
        AHRS_MessageQueue& ahrsMessageQueue;
        AltitudeMessageQueue& altitudeMessageQueue;
        BarometerBase& barometer;
    };
public:
    AltitudeTask(uint32_t taskIntervalMicroseconds, const parameters_t& parameters) :
        TaskBase(taskIntervalMicroseconds),
        _altitudeKalmanFilter(parameters.altitudeKalmanFilter),
        _ahrsMessageQueue(parameters.ahrsMessageQueue),
        _altitudeMessageQueue(parameters.altitudeMessageQueue),
        _barometer(parameters.barometer)
    {}
public:
    static AltitudeTask* createTask(task_info_t& taskInfo, const parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static AltitudeTask* createTask(const parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
public:
    [[noreturn]] static void Task(void* arg);
private:
    [[noreturn]] void task();
private:
    AltitudeKalmanFilter& _altitudeKalmanFilter;
    AHRS_MessageQueue& _ahrsMessageQueue;
    AltitudeMessageQueue& _altitudeMessageQueue;
    BarometerBase& _barometer;
};
