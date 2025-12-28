#include "Main.h"

#include "NonVolatileStorage.h"
#include <VTX.h>


/*!
Statically allocate the VTX and load its default configuration.
*/
VTX* Main::createVTX(NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_VTX)
    static VTX vtx;
    vtx.setConfig(nvs.loadVTX_Config());

    return &vtx;
#else
    (void)nvs;
    return nullptr;
#endif
}
