#pragma once

#include <task_base.h>

class OSD;

struct osd_context_t;


class OSD_Task : public TaskBase {
public:
    OSD_Task(uint32_t task_interval_microseconds, OSD& osd, osd_context_t& context) :
        TaskBase(task_interval_microseconds),
        _osd(osd),
        _context(context)
        {}
public:
    static OSD_Task* create_task(task_info_t& task_info, OSD& osd, osd_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static OSD_Task* create_task(OSD& osd, osd_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    OSD& _osd;
    osd_context_t& _context;
};
