#include "MSP_Protoflight.h"
#include "MSP_SerialPort.h"
#include "Main.h"
#include <serial_port.h>

#include <msp_serial.h>
#include <msp_stream.h>


/*!
Statically allocate the MSP and associated objects.
*/
MspSerial* Main::createMSP(MspBase*& msp_base)
{
#if defined(USE_MSP)
    static MSP_Protoflight mspProtoflight;
    msp_base = & mspProtoflight;
    static MspStream mspStream(mspProtoflight);
    enum { BAUD_RATE = 115200 };
    enum { DATA_BITS = 8, PARITY = SerialPort::PARITY_NONE, STOP_BITS = 1 }; // 8N1
    static SerialPort serialPort(SerialPort::MSP_UART_PINS, MSP_UART_INDEX, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY);
    static MSP_SerialPort mspSerialPort(serialPort);
    static MspSerial mspSerial(mspStream, mspSerialPort);
    return &mspSerial;
#else
    msp_base = nullptr;
    return nullptr;
#endif
}
