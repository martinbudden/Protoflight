#include "ButtonsBase.h"
#include "Dashboard.h"
#include "DisplayPortBase.h"
#include "ScreenBase.h"


Dashboard::Dashboard(ScreenBase* screen, ButtonsBase* buttons) :
    _screen(screen),
    _buttons(buttons)
{
}

void Dashboard::updateDashboard(dashboard_parameter_group_t& pg)
{
    if (pg.displayPort.isGrabbed()) {
        return;
    }
    if (_screen) {
        _screen->update(pg.ahrsMessageQueue, pg.motorMixer, pg.receiver);
    }
    if (_buttons) {
        _buttons->update(pg.flightController, pg.motorMixer, pg.receiver);
    }
}
