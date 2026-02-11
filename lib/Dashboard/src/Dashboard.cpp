#include "ButtonsBase.h"
#include "Dashboard.h"
#include "ScreenBase.h"


Dashboard::Dashboard(FlightController& flightController, const ReceiverBase& receiver, ScreenBase* screen, ButtonsBase* buttons) :
    _flightController(flightController),
    _receiver(receiver),
    _screen(screen),
    _buttons(buttons)
{
}

void Dashboard::updateDashboard()
{
    if (_screen) {
        _screen->update(_flightController, _receiver);
    }
    if (_buttons) {
        _buttons->update(_flightController, _receiver);
    }
}
