#include "main.h"

#include <cms.h>


/*!
Statically allocate the CMS.
*/
CMS* Main::create_cms()
{
#if defined(USE_CMS)
    static CMS cms;

    return &cms;
#else
    return nullptr;
#endif
}
