#include "Cockpit.h"
#include "DisplayPortBase.h"
#include "Main.h"
#include "NonVolatileStorage.h"
#include "OSD.h"


/*!
Statically allocate the OSD and load its default configuration.
*/
OSD* Main::createOSD(DisplayPortBase& displayPort, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_OSD)
    static OSD osd;
    osd.init(&displayPort);
    osd.setConfig(nvs.load_osd_config());

    OSD_Elements& osdElements = osd.getOSD_Elements();
    if (!nvs.load_osd_elements_config(osdElements.getConfig())) {
        osdElements.setDefaultConfig(displayPort.getRowCount(), displayPort.getColumnCount());
    }
    return &osd;
#else
    (void)displayPort;
    (void)nvs;
    return nullptr;
#endif
}
