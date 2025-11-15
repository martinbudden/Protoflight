#include "Main.h"

#include <NonVolatileStorage.h>
#include <OSD.h>


/*!
Statically allocate the OSD and associated objects.
*/
OSD& Main::createOSD(DisplayPortBase* displayPort, const FlightController& flightController, const Cockpit& cockpit, Debug& debug, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
    static OSD osd(flightController, cockpit, debug);

#if defined(USE_OSD)
    osd.init(displayPort, DisplayPortBase::DEVICE_TYPE_OLED);
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
