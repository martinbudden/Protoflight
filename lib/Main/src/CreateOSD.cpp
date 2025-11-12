#include "Main.h"

#include <DisplayPortM5GFX.h>
#include <DisplayPortNull.h>
#include <NonVolatileStorage.h>
#include <OSD.h>


/*!
Statically allocate the OSD and associated objects.
*/
OSD& Main::createOSD(const FlightController& flightController, const Cockpit& cockpit, Debug& debug, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(M5_UNIFIED)
    static M5Canvas canvas(&M5.Display);
    static DisplayPortM5GFX displayPort(canvas);
#else
    static DisplayPortNull displayPort;
#endif

    static OSD osd(flightController, cockpit, debug);

#if defined(USE_OSD)
    osd.init(&displayPort, DisplayPortBase::DEVICE_TYPE_NONE);
    osd.setConfig(nvs.loadOSD_Config());

    OSD_Elements& osdElements = osd.getOSD_Elements();
    if (!nvs.loadOSD_ElementsConfig(osdElements.getConfig())) {
        osdElements.setDefaultConfig();
    }
#else
    (void)nvs;
    (void)displayPort;
#endif

    return osd;
}
