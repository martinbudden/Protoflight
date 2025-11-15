#pragma once

class ButtonsBase;
class ScreenBase;


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
    void updateDashboard(); //!< Dashboard Task function, called by Task
    ScreenBase* getScreen() const { return _screen; }
private:
    ScreenBase* _screen {nullptr};
    ButtonsBase* _buttons {nullptr};
};
