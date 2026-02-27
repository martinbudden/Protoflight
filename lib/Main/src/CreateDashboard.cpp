#include "Main.h"

#if defined(M5_UNIFIED)
#include "ButtonsM5.h"
#endif
#include "Dashboard.h"
#if defined(M5_UNIFIED)
#include "ScreenM5.h"
#endif


Dashboard* Main::createDashboard(const ReceiverBase& receiver)
{
#if defined(USE_DASHBOARD)
    // Statically allocate the screen.
    static ScreenM5 screen;
    screen.updateTemplate(receiver); // Update the screen as soon as we can, to minimize the time the screen is blank
    // Statically allocate the buttons.
    static ButtonsM5 buttons(&screen);
    static Dashboard dashboard(&screen, &buttons);

    return &dashboard;
#else
    (void)receiver;

    return nullptr;
#endif
}
