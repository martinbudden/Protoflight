#pragma once

#include <TaskBase.h>

class OSD;


class OSD_Task : public TaskBase {
public:
    OSD_Task(uint32_t taskIntervalMicroseconds, OSD& osd) :
        TaskBase(taskIntervalMicroseconds),
        _osd(osd) {}
public:
    static OSD_Task* createTask(task_info_t& taskInfo, OSD& osd, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static OSD_Task* createTask(OSD& osd, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    OSD& _osd;
};
