#include "main.h"
#include <gps.h>
#include <gps_message_queue.h>
#include <serial_port.h>


/*!
Statically allocate the GPS.
*/
GPS* Main::create_gps()
{
#if defined(USE_GPS)
    static SerialPort serial_port(SerialPort::GPS_UART_PINS, GPS_UART_INDEX, SerialPort::BAUDRATE_9600, SerialPort::DATA_BITS_8, SerialPort::STOP_BITS_1, SerialPort::PARITY_NONE);
    static GPS_MessageQueue gpsMessageQueue;
    static GPS gps(serial_port, gpsMessageQueue);
    return &gps;
#else
    return nullptr;
#endif
}
