#pragma once

class AHRS;
class ButtonsBase;
class FlightController;
class ReceiverBase;
class ScreenBase;


class Dashboard {
public:
    Dashboard(FlightController& flightController, const ReceiverBase& _receiver, ScreenBase* screen, ButtonsBase* buttons);
private:
    // Dashboard is not copyable or moveable
    Dashboard(const Dashboard&) = delete;
    Dashboard& operator=(const Dashboard&) = delete;
    Dashboard(Dashboard&&) = delete;
    Dashboard& operator=(Dashboard&&) = delete;
public:
    void updateDashboard(); //!< Dashboard Task function, called by Task
    ScreenBase* getScreen() const { return _screen; }
private:
    FlightController& _flightController;
    const ReceiverBase& _receiver;
    ScreenBase* _screen {nullptr};
    ButtonsBase* _buttons {nullptr};
};
