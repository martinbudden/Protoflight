#include "Main.h"

#include "NonVolatileStorage.h"
#include <VTX_Base.h>


/*!
Statically allocate the VTX and load its default configuration.
*/
VTX_Base* Main::createVTX(NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_VTX)
    static VTX_Base vtx;
    vtx.setConfig(nvs.loadVTXConfig());

    return &vtx;
#else
    (void)nvs;
    return nullptr;
#endif
}
