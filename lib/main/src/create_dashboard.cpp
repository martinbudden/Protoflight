#include "main.h"

#if defined(M5_UNIFIED)
#include "buttons_m5.h"
#endif
#include "dashboard.h"
#if defined(M5_UNIFIED)
#include "screen_m5.h"
#endif


Dashboard* Main::create_dashboard(const ReceiverBase& receiver)
{
#if defined(USE_DASHBOARD) && defined(M5_UNIFIED)
    // Statically allocate the screen.
    static ScreenM5 screen;
    screen.update_template(receiver); // Update the screen as soon as we can, to minimize the time the screen is blank
    // Statically allocate the buttons.
    static ButtonsM5 buttons(&screen);
    static Dashboard dashboard(&screen, &buttons);

    return &dashboard;
#else
    (void)receiver;

    return nullptr;
#endif
}
