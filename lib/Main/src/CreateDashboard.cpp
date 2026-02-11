#include "Main.h"

#if defined(M5_UNIFIED)
#include "ButtonsM5.h"
#endif
#include "Dashboard.h"
#if defined(M5_UNIFIED)
#include "ScreenM5.h"
#endif


Dashboard* Main::createDashboard(const DisplayPortBase& displayPort, FlightController& flightController, const ReceiverBase& receiver)
{
#if defined(USE_DASHBOARD)
    // Statically allocate the screen.
    static ScreenM5 screen(displayPort);
    screen.updateTemplate(receiver); // Update the screen as soon as we can, to minimize the time the screen is blank
    // Statically allocate the buttons.
    static ButtonsM5 buttons(&screen);
    static Dashboard dashboard(flightController, receiver, &screen, &buttons);

    return &dashboard;
#else
    (void)displayPort;
    (void)flightController;
    (void)receiver;

    return nullptr;
#endif
}
