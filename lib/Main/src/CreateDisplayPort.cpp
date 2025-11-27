#include "DisplayPortBase.h"
#include "DisplayPortM5GFX.h"
#include "DisplayPortMSP.h"
#include "DisplayPortMax7456.h"
#include "DisplayPortNull.h"
#include "Main.h"

#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#include <cassert>


/*!
Statically allocate the DisplayPort.
*/
DisplayPortBase& Main::createDisplayPort([[maybe_unused]] Debug& debug, [[maybe_unused]] MSP_SerialBase* mspSerial)
{
#if defined(M5_UNIFIED)
    static M5Canvas canvas(&M5.Display);
    static DisplayPortM5GFX displayPort(canvas, 320, 240);
#elif defined(USE_MAX7456)
    static DisplayPortMax7456 displayPort(BUS_SPI::MAX7456_SPI_INDEX, BUS_SPI::MAX7456_SPI_PINS, debug);
#elif defined(USE_MSP_DISPLAYPORT)
    assert(mspSerial != nullptr);
    static DisplayPortMSP displayPort(*mspSerial);
#else
    static DisplayPortNull displayPort;
#endif

    return displayPort;
}
