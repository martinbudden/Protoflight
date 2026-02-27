#include "DisplayPortBase.h"
#include "DisplayPortM5GFX.h"
#include "DisplayPortMSP.h"
#include "DisplayPortMax7456.h"
#include "DisplayPortNull.h"

#include "MSP_SerialPort.h"
#include "Main.h"
#include <serial_port.h>

#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#include <MSP_Serial.h>
#include <MSP_Stream.h>

#include <cassert>


/*!
Statically allocate the DisplayPort.
*/
DisplayPortBase& Main::createDisplayPort()
{
#if defined(M5_UNIFIED)
    static M5Canvas canvas(&M5.Display);
    static DisplayPortM5GFX displayPort(canvas, 320, 240);
#elif defined(USE_MAX7456)
    static DisplayPortMax7456 displayPort(BusSpi::MAX7456_SPI_INDEX, BusSpi::MAX7456_SPI_PINS);
#elif defined(USE_MSP_DISPLAYPORT)
    enum { BAUD_RATE = 115200 };
    enum { DATA_BITS = 8, PARITY = SerialPort::PARITY_NONE, STOP_BITS = 1 }; // 8N1
    static SerialPort serialPort(SerialPort::MSP_DISPLAYPORT_UART_PINS, MSP_DISPLAYPORT_UART_INDEX, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY);
    static MSP_SerialPort mspSerialPort(serialPort);
    static MSP_Base mspBase;
    static MSP_Stream mspStream(mspBase);
    static MSP_Serial mspSerial(mspStream, mspSerialPort);
    static DisplayPortMSP displayPort(mspStream);
#else
    static DisplayPortNull displayPort;
#endif

    return displayPort;
}
