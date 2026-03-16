#include "display_port_base.h"
#include "display_port_m5gfx.h"
#include "display_port_max7456.h"
#include "display_port_msp.h"
#include "display_port_null.h"

#include "main.h"
#include "msp_serial_port.h"

#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#include <cassert>

#include <msp_serial.h>
#include <msp_stream.h>
#include <serial_port.h>


/*!
Statically allocate the DisplayPort.
*/
DisplayPortBase& Main::create_display_port()
{
#if defined(M5_UNIFIED)
    static M5Canvas canvas(&M5.Display);
    static DisplayPortM5GFX display_port(canvas, 320, 240);
#elif defined(USE_MAX7456)
    static DisplayPortMax7456 display_port(BusSpi::MAX7456_SPI_INDEX, BusSpi::MAX7456_SPI_PINS);
#elif defined(USE_MSP_DISPLAYPORT)
    enum { BAUD_RATE = 115200 };
    enum { DATA_BITS = 8, PARITY = SerialPort::PARITY_NONE, STOP_BITS = 1 }; // 8N1
    static SerialPort serial_port(SerialPort::MSP_DISPLAYPORT_UART_PINS, MSP_DISPLAYPORT_UART_INDEX, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY);
    static MSP_SerialPort mspSerialPort(serial_port);
    static MSP_Base mspBase;
    static MSP_Stream msp_stream(mspBase);
    static MSP_Serial mspSerial(msp_stream, mspSerialPort);
    static DisplayPortMSP display_port(msp_stream);
#else
    static DisplayPortNull display_port;
#endif

    return display_port;
}
