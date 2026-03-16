#pragma once

#include <task_base.h>

class CMS;

struct cms_context_t;


class CMS_Task : public TaskBase {
public:
    CMS_Task(uint32_t task_interval_microseconds, CMS& cms, cms_context_t& context) :
        TaskBase(task_interval_microseconds),
        _cms(cms),
        _context(context)
        {}
public:
    static CMS_Task* create_task(task_info_t& task_info, CMS& cms, cms_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static CMS_Task* create_task(CMS& cms, cms_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
public:
    [[noreturn]] static void task_static(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    CMS& _cms;
    cms_context_t& _context;
};
