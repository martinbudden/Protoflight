#include "Main.h"
#include <GPS.h>
#include <GPS_MessageQueue.h>
#include <SerialPort.h>


/*!
Statically allocate the GPS.
*/
GPS* Main::createGPS(Debug& debug)
{
#if defined(USE_GPS)
    static SerialPort serialPort(SerialPort::GPS_UART_PINS, GPS_UART_INDEX, SerialPort::BAUDRATE_9600, SerialPort::DATA_BITS_8, SerialPort::STOP_BITS_1, SerialPort::PARITY_NONE);
    static GPS_MessageQueue gpsMessageQueue;
    static GPS gps(serialPort, gpsMessageQueue, debug);
    return &gps;
#else
    (void)debug;
    return nullptr;
#endif
}
