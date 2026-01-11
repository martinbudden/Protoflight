#pragma once

#include "GPS_MessageData.h"

#include <SerialPort.h>

#include <array>
#include <cstdint>

class Debug;
class GPS_MessageQueue;
class UBLOX;


/*!
*/
class GPS {
public:
    virtual ~GPS() = default;
    GPS(SerialPort& serialPort, GPS_MessageQueue& gpsMessageQueue, Debug& debug) : _serialPort(serialPort), _gpsMessageQueue(gpsMessageQueue), _debug(debug) {}
    void init();
    GPS_MessageQueue& getGPS_MessageQueue() { return _gpsMessageQueue; }
private:
    // GPS is not copyable or moveable
    GPS(const GPS&) = delete;
    GPS& operator=(const GPS&) = delete;
    GPS(GPS&&) = delete;
    GPS& operator=(GPS&&) = delete;
public:
    enum state_flags_e {
        FIX_HOME   = 0x01,
        FIX        = 0x02,
        FIX_EVER   = 0x04,
    };
    enum model_e {
        MODEL_PORTABLE = 0,
        MODEL_STATIONARY,
        MODEL_PEDESTRIAN,
        MODEL_AUTOMOTIVE,
        MODEL_AT_SEA,
        MODEL_AIRBORNE_1G,
        MODEL_AIRBORNE_2G,
        MODEL_AIRBORNE_4G,
    };
    enum utc_standard_e {
        UTC_STANDARD_AUTO = 0,
        UTC_STANDARD_USNO = 3,
        UTC_STANDARD_EU = 5,
        UTC_STANDARD_SU = 6,
        UTC_STANDARD_NTSC = 7,
    };
    enum sbas_mode_e { // Satellite-based augmentation system
        SBAS_AUTO = 0,
        SBAS_EGNOS,
        SBAS_WAAS,
        SBAS_MSAS,
        SBAS_GAGAN,
        SBAS_NONE
    };
    enum auto_config_e {
        AUTO_CONFIG_OFF = 0,
        AUTO_CONFIG_ON
    };
    enum auto_baud_e {
        AUTO_BAUD_OFF = 0,
        AUTO_BAUD_ON
    };
    enum provider_e {
        GPS_NMEA = 0,
        GPS_UBLOX,
        GPS_MSP,
        GPS_VIRTUAL,
    };
    struct config_t {
        uint8_t provider;
        uint8_t sbasMode;
        uint8_t autoConfig;
        uint8_t autoBaud;
        uint8_t gps_ublox_acquire_model;
        uint8_t gps_ublox_flight_model;
        uint8_t gps_update_rate_hz;
        bool gps_ublox_use_galileo;
        bool gps_set_home_point_once;
        bool gps_use_3d_speed;
        bool sbas_integrity;
        uint8_t gps_ublox_utc_standard;
    };
private:
    SerialPort& _serialPort;
    GPS_MessageQueue& _gpsMessageQueue;
    Debug& _debug;
    config_t _config {};
};
