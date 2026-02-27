#include "Main.h"

#include <CMS.h>


/*!
Statically allocate the CMS.
*/
CMS* Main::createCMS()
{
#if defined(USE_CMS)
    static CMS cms;

    return &cms;
#else
    return nullptr;
#endif
}
