#pragma once

#include <TaskBase.h>

class Dashboard;


class DashboardTask : public TaskBase {
public:
    DashboardTask(uint32_t taskIntervalMicroseconds, Dashboard& dashboard) :
        TaskBase(taskIntervalMicroseconds),
        _dashboard(dashboard) {}
public:
    static DashboardTask* createTask(task_info_t& taskInfo, Dashboard& dashboard, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static DashboardTask* createTask(Dashboard& dashboard, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    Dashboard& _dashboard;
};
