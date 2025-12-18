#include "DisplayPortBase.h"
#include "Main.h"
#include "NonVolatileStorage.h"
#include "OSD.h"


/*!
Statically allocate the OSD and load its default configuration.
*/
OSD* Main::createOSD(DisplayPortBase& displayPort, const FlightController& flightController, const Cockpit& cockpit, Debug& debug, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_OSD)
    static OSD osd(flightController, cockpit, flightController.getAHRS_MessageQueue(), debug);
    osd.init(&displayPort);
    osd.setConfig(nvs.loadOSD_Config());

    OSD_Elements& osdElements = osd.getOSD_Elements();
    if (!nvs.loadOSD_ElementsConfig(osdElements.getConfig())) {
        osdElements.setDefaultConfig(displayPort.getRowCount(), displayPort.getColumnCount());
    }
    return &osd;
#else
    (void)displayPort;
    (void)flightController;
    (void)cockpit;
    (void)debug;
    (void)nvs;
    return nullptr;
#endif
}
