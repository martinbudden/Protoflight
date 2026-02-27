#pragma once

#include <task_base.h>

class OSD;

struct osd_parameter_group_t;


class OSD_Task : public TaskBase {
public:
    OSD_Task(uint32_t task_interval_microseconds, OSD& osd, osd_parameter_group_t& parameter_group) :
        TaskBase(task_interval_microseconds),
        _osd(osd),
        _parameter_group(parameter_group)
        {}
public:
    static OSD_Task* create_task(task_info_t& task_info, OSD& osd, osd_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static OSD_Task* create_task(OSD& osd, osd_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    OSD& _osd;
    osd_parameter_group_t& _parameter_group;
};
