#include "main.h"

#include "non_volatile_storage.h"
#include <vtx.h>


/*!
Statically allocate the VTX and load its default configuration.
*/
VTX* Main::create_vtx(NonVolatileStorage& nvs) // cppcheck-suppress constParameterReference
{
#if defined(USE_VTX)
    static VTX vtx;
    vtx.set_config(nvs.load_vtx_config());

    return &vtx;
#else
    (void)nvs;
    return nullptr;
#endif
}
