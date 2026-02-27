#pragma once

class Ahrs;
class AhrsMessageQueue;
class ButtonsBase;
class DisplayPortBase;
class FlightController;
class MotorMixerBase;
class ReceiverBase;
class ScreenBase;

struct dashboard_parameter_group_t {
    const DisplayPortBase& displayPort;
    FlightController& flightController;
    const AhrsMessageQueue& ahrsMessageQueue;
    MotorMixerBase& motorMixer;
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
    void updateDashboard(dashboard_parameter_group_t& pg); //!< Dashboard Task function, called by Task
    ScreenBase* getScreen() const { return _screen; }
private:
    ScreenBase* _screen {nullptr};
    ButtonsBase* _buttons {nullptr};
};
