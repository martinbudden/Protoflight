#include "ButtonsBase.h"
#include "Dashboard.h"
#include "ScreenBase.h"


Dashboard::Dashboard(ScreenBase* screen, ButtonsBase* buttons) :
    _screen(screen),
    _buttons(buttons)
{
}

void Dashboard::updateDashboard()
{
    if (_screen) {
        _screen->update();
    }
    if (_buttons) {
        _buttons->update();
    }
}
