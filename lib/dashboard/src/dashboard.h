#pragma once

#include <cstdint>

class Ahrs;
class AhrsMessageQueue;
class ButtonsBase;
class DisplayPortBase;
class FlightController;
class MotorMixerBase;
class ReceiverBase;
class ScreenBase;

struct dashboard_context_t {
    const DisplayPortBase& display_port;
    FlightController& flight_controller;
    const AhrsMessageQueue& ahrs_message_queue;
    MotorMixerBase& motor_mixer;
    const ReceiverBase& receiver;
};


class Dashboard {
public:
    Dashboard(ScreenBase* screen, ButtonsBase* buttons);
private:
    // Dashboard is not copyable or moveable
    Dashboard(const Dashboard&) = delete;
    Dashboard& operator=(const Dashboard&) = delete;
    Dashboard(Dashboard&&) = delete;
    Dashboard& operator=(Dashboard&&) = delete;
public:
    void update_dashboard(uint32_t tick_count, dashboard_context_t& ctx); //!< Dashboard Task function, called by Task
    ScreenBase* get_screen() const { return _screen; }
private:
    ScreenBase* _screen {nullptr};
    ButtonsBase* _buttons {nullptr};
    uint32_t _screen_tick_count {0};
    uint32_t _buttons_tick_count {0};
};
