#include "main.h"
#include "msp_protoflight.h"
#include "msp_serial_port.h"

#include <msp_serial.h>
#include <msp_stream.h>
#include <serial_port.h>


/*!
Statically allocate the MSP and associated objects.
*/
MspSerial* Main::create_msp(MspBase*& msp_base_ptr)
{
#if defined(USE_MSP)
    static MSP_Protoflight mspProtoflight;
    msp_base_ptr = &mspProtoflight;
    static MspStream msp_stream(mspProtoflight);
    enum { BAUD_RATE = 115200 };
    enum { DATA_BITS = 8, PARITY = SerialPort::PARITY_NONE, STOP_BITS = 1 }; // 8N1
    static SerialPort serial_port(SerialPort::MSP_UART_PINS, MSP_UART_INDEX, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY);
    static MSP_SerialPort mspSerialPort(serial_port);
    static MspSerial mspSerial(msp_stream, mspSerialPort);
    return &mspSerial;
#else
    msp_base_ptr = nullptr;
    return nullptr;
#endif
}
