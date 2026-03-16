#pragma once

#include <task_base.h>

class Dashboard;

struct dashboard_context_t;


class DashboardTask : public TaskBase {
public:
    DashboardTask(uint32_t task_interval_microseconds, Dashboard& dashboard, dashboard_context_t& context) :
        TaskBase(task_interval_microseconds),
        _dashboard(dashboard),
        _context(context)
        {}
public:
    static DashboardTask* create_task(task_info_t& task_info, Dashboard& dashboard, dashboard_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static DashboardTask* create_task(Dashboard& dashboard, dashboard_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
public:
    [[noreturn]] static void task_static(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    Dashboard& _dashboard;
    dashboard_context_t& _context;
};
