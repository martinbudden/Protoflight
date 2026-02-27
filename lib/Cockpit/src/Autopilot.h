#pragma once

#include "FlightController.h"

#include <receiver_base.h>
#if defined(USE_GPS)
#include <GeographicCoordinate.h>
#endif


class AltitudeMessageQueue;

#if !defined(MAX_WAYPOINT_COUNT)
enum { MAX_WAYPOINT_COUNT = 16 };
#endif


struct autopilot_config_t {
    VehicleControllerBase::PIDF_uint16_t altitudePID;
    VehicleControllerBase::PIDF_uint16_t positionPID;
    uint16_t landingAltitudeMeters;
    uint16_t throttle_hover_pwm;
    uint16_t throttle_min_pwm;
    uint16_t throttle_max_pwm;
    uint8_t position_lpf_hz100; // cutoff frequency*100 for longitude and latitude position filters
    uint8_t maxAngle;
};

struct gps_rescue_config_t {
    uint16_t maxRescueAngle_degrees;
    uint16_t returnAltitude_meters;
    uint16_t descentDistance_meters;
    uint16_t ground_speed_cmps; // centimeters per second
    uint8_t  yawP;
    uint8_t  minSats;
    uint8_t  velP;
    uint8_t  velI;
    uint8_t  velD;
    uint16_t minStartDist_meters;
    uint8_t  sanityChecks;
    uint8_t  allowArmingWithoutFix;
    uint8_t  useMag;
    uint8_t  altitudeMode;
    uint16_t ascendRate;
    uint16_t descendRate;
    uint16_t initialClimb_meters;
    uint8_t  rollMix;
    uint8_t  disarmThreshold;
    uint8_t  pitchCutoffHz;
    uint8_t  imuYawGain;
};

struct position_hold_config_t {
    uint16_t altitude_lpf_hz100;   // lowpass cutoff Hz*100 for altitude smoothing
    uint16_t altitude_dterm_lpf_hz100; // lowpass cutoff Hz*100 for altitude derivative smoothing
    uint8_t altitude_source;
    uint8_t altitude_prefer_baro; // percentage trust of barometer data
};

struct altitude_hold_config_t {
    uint8_t climbRate;
    uint8_t deadband;
};


class Autopilot {
public:
    virtual ~Autopilot() = default;
    Autopilot(const AhrsMessageQueue& ahrsMessageQueue) : _ahrsMessageQueue(ahrsMessageQueue) {}
    Autopilot(const AhrsMessageQueue& ahrsMessageQueue, AltitudeMessageQueue& altitudeMessageQueue) : _ahrsMessageQueue(ahrsMessageQueue), _altitudeMessageQueue(&altitudeMessageQueue) {}
private:
    // Autopilot is not copyable or moveable
    Autopilot(const Autopilot&) = delete;
    Autopilot& operator=(const Autopilot&) = delete;
    Autopilot(Autopilot&&) = delete;
    Autopilot& operator=(Autopilot&&) = delete;
public:
    enum earth_frame_axis_e { LONGITUDE, LATITUDE, EARTH_FRAME_AXIS_COUNT };
    enum altitude_source_e { DEFAULT_SOURCE = 0, BAROMETER_ONLY, GPS_ONLY };
    struct earth_frame_t {
        PowerTransferFilter1 velocityLPF;
        PowerTransferFilter1 accelerationLPF;
        PidController pid;
    };
    struct altitude_t {
        PowerTransferFilter1 altitudeLPF;
        PowerTransferFilter2 dTermLPF;
        PidController pid;
        float hoverThrottle;
    };
public:
    void set_autopilot_config(const autopilot_config_t& autopilotConfig);
    const autopilot_config_t& get_autopilot_config() const { return _autopilotConfig; }
#if defined(USE_GPS)
    void setGPS_RescueConfig(const gps_rescue_config_t& gpsRescueConfig);
    const gps_rescue_config_t& getGPS_RescueConfig() const { return _gpsRescueConfig; }
#endif
    void setPositionHoldConfig(const position_hold_config_t& positionHoldConfig);
    const position_hold_config_t& getPositionHoldConfig() const { return _positionHoldConfig; }
    void set_altitude_hold_config(const altitude_hold_config_t& altitudeHoldConfig);
    const altitude_hold_config_t& get_altitude_hold_config() const { return _altitudeHoldConfig; }
    bool isAltitudeHoldSetpointSet() const; //!< returns true if setpoint has been set
    bool setAltitudeHoldSetpoint(); //!< use the current altitude to set the setpoint for altitude hold
    float calculateThrottleForAltitudeHold(const receiver_controls_t& controls);
    FlightController::controls_t calculateFlightControls(const receiver_controls_t& controls, uint32_t flightModeModeFlags);

    AltitudeMessageQueue* getAltitudeMessageQueueMutable() { return _altitudeMessageQueue; }
    const AltitudeMessageQueue* getAltitudeMessageQueue() const { return _altitudeMessageQueue; }
private:
    const AhrsMessageQueue& _ahrsMessageQueue;
    AltitudeMessageQueue* _altitudeMessageQueue {nullptr};
    altitude_t _altitude {};
    std::array<earth_frame_t, EARTH_FRAME_AXIS_COUNT> _earthFrames {};
    autopilot_config_t _autopilotConfig;
    position_hold_config_t _positionHoldConfig;
    altitude_hold_config_t _altitudeHoldConfig;
    // all positions are specified in meters from home position
    xyz_t _currentPositionMeters {};
    xyz_t _targetPositionMeters {};
#if defined(USE_GPS)
    gps_rescue_config_t _gpsRescueConfig {};
    // waypoints are specified as latitude/longitude/altitude coordinates
    std::array<geographic_coordinate_t, MAX_WAYPOINT_COUNT> _waypoints;
#endif
};
