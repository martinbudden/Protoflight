#include "MSP_Protoflight.h"
#include "MSP_SerialPort.h"
#include "Main.h"
#include "SerialPort.h"

#include <MSP_Serial.h>
#include <MSP_Stream.h>


/*!
Statically allocate the MSP and associated objects.
*/
MSP_Serial* Main::createMSP(AHRS& ahrs, FlightController& flightController, Cockpit& cockpit, const ReceiverBase& receiver, const IMU_Filters& imuFilters, Debug& debug, NonVolatileStorage& nvs, Blackbox* blackbox, VTX* vtx, OSD* osd)
{
#if defined(USE_MSP)
    static MSP_Protoflight mspProtoflight(ahrs, flightController, cockpit, receiver, imuFilters, debug, nvs, blackbox, vtx, osd);
    static MSP_Stream mspStream(mspProtoflight);
    enum { BAUD_RATE = 115200 };
    enum { DATA_BITS = 8, PARITY = SerialPort::PARITY_NONE, STOP_BITS = 1 }; // 8N1
    static SerialPort serialPort(SerialPort::MSP_UART_PINS, MSP_UART_INDEX, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY);
    static MSP_SerialPort mspSerialPort(serialPort);
    static MSP_Serial mspSerial(mspStream, mspSerialPort);
    return &mspSerial;
#else
    (void)ahrs;
    (void)flightController;
    (void)cockpit;
    (void)receiver;
    (void)imuFilters;
    (void)debug;
    (void)nvs;
    (void)blackbox;
    (void)vtx;
    (void)osd;

    return nullptr;
#endif
}
