#pragma once

#include <TaskBase.h>

class CMS;


class CMS_Task : public TaskBase {
public:
    CMS_Task(uint32_t taskIntervalMicroseconds, CMS& cms) :
        TaskBase(taskIntervalMicroseconds),
        _cms(cms) {}
public:
    static CMS_Task* createTask(task_info_t& taskInfo, CMS& cms, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static CMS_Task* createTask(CMS& cms, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    CMS& _cms;
};
