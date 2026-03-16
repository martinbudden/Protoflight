#include "cockpit.h"
#include "display_port_base.h"
#include "main.h"
#include "non_volatile_storage.h"
#include "osd.h"


/*!
Statically allocate the OSD and load its default configuration.
*/
OSD* Main::create_osd(DisplayPortBase& display_port, NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_OSD)
    static OSD osd;
    osd.init(&display_port);
    osd.set_config(nvs.load_osd_config());

    OSD_Elements& osdElements = osd.get_osd_elements();
    if (!nvs.load_osd_elements_config(osdElements.get_config())) {
        osdElements.set_default_config(display_port.get_row_count(), display_port.get_column_count());
    }
    return &osd;
#else
    (void)display_port;
    (void)nvs;
    return nullptr;
#endif
}
