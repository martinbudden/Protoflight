#include "DisplayPortBase.h"
#include "DisplayPortM5GFX.h"
#include "DisplayPortMax7456.h"
#include "DisplayPortNull.h"
#include "Main.h"

#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif


/*!
Statically allocate the DisplayPort.
*/
DisplayPortBase& Main::createDisplayPort(Debug& debug)
{
#if defined(M5_UNIFIED)
    (void)debug;
    static M5Canvas canvas(&M5.Display);
    static DisplayPortM5GFX displayPort(canvas, 320, 240);
#elif defined(USE_MAX7456)
    static DisplayPortMax7456 displayPort(BUS_SPI::MAX7456_SPI_INDEX, BUS_SPI::MAX7456_SPI_PINS, debug);
#else
    (void)debug;
    static DisplayPortNull displayPort;
#endif

    return displayPort;
}
