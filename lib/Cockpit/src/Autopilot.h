#pragma once

#include "FlightController.h"

#include <CockpitBase.h>
#if defined(USE_GPS)
#include <GeographicCoordinate.h>
#endif


class AltitudeMessageQueue;

#if !defined(MAX_WAYPOINT_COUNT)
enum { MAX_WAYPOINT_COUNT = 16 };
#endif


class Autopilot {
public:
    virtual ~Autopilot() = default;
    Autopilot(const AHRS_MessageQueue& ahrsMessageQueue) : _ahrsMessageQueue(ahrsMessageQueue) {}
    Autopilot(const AHRS_MessageQueue& ahrsMessageQueue, AltitudeMessageQueue& altitudeMessageQueue) : _ahrsMessageQueue(ahrsMessageQueue), _altitudeMessageQueue(&altitudeMessageQueue) {}
private:
    // Autopilot is not copyable or moveable
    Autopilot(const Autopilot&) = delete;
    Autopilot& operator=(const Autopilot&) = delete;
    Autopilot(Autopilot&&) = delete;
    Autopilot& operator=(Autopilot&&) = delete;
public:
    enum earth_frame_axis_e { LONGITUDE, LATITUDE, EARTH_FRAME_AXIS_COUNT };
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
    enum altitude_source_e { DEFAULT_SOURCE = 0, BAROMETER_ONLY, GPS_ONLY };
    struct position_config_t {
        uint16_t altitude_lpf_hz100;   // lowpass cutoff Hz*100 for altitude smoothing
        uint16_t altitude_dterm_lpf_hz100; // lowpass cutoff Hz*100 for altitude derivative smoothing
        uint8_t altitude_source;
        uint8_t altitude_prefer_baro; // percentage trust of barometer data
    };
    struct altitude_hold_config_t {
        uint8_t climbRate;
        uint8_t deadband;
    };
    struct earth_frame_t {
        PowerTransferFilter1 velocityLPF;
        PowerTransferFilter1 accelerationLPF;
        PIDF pid;
    };
    struct altitude_t {
        PowerTransferFilter1 altitudeLPF;
        PowerTransferFilter2 dTermLPF;
        PIDF pid;
        float hoverThrottle;
    };
public:
    void setAutopilotConfig(const autopilot_config_t& autopilotConfig);
    const autopilot_config_t& getAutopilotConfig() const { return _autopilotConfig; }
    void setPositionConfig(const position_config_t& positionConfig);
    const position_config_t& getPositionConfig() const { return _positionConfig; }
    void setAltitudeHoldConfig(const altitude_hold_config_t& altitudeHoldConfig);
    const altitude_hold_config_t& getAltitudeHoldConfig() const { return _altitudeHoldConfig; }

    bool isAltitudeHoldSetpointSet() const; //!< returns true if setpoint has been set
    bool setAltitudeHoldSetpoint(); //!< use the current altitude to set the setpoint for altitude hold
    float calculateThrottleForAltitudeHold(const CockpitBase::controls_t& controls);
    FlightController::controls_t calculateFlightControls(const CockpitBase::controls_t& controls, uint32_t flightModeModeFlags);

    AltitudeMessageQueue* getAltitudeMessageQueue() { return _altitudeMessageQueue; }
    //const AltitudeMessageQueue* getAltitudeMessageQueue() const { return _altitudeMessageQueue; }
private:
    const AHRS_MessageQueue& _ahrsMessageQueue;
    AltitudeMessageQueue* _altitudeMessageQueue {nullptr};
    altitude_t _altitude {};
    std::array<earth_frame_t, EARTH_FRAME_AXIS_COUNT> _earthFrames {};
    autopilot_config_t _autopilotConfig;
    position_config_t _positionConfig;
    altitude_hold_config_t _altitudeHoldConfig;
    // all positions are specified in meters from home position
    xyz_t _currentPositionMeters {};
    xyz_t _targetPositionMeters {};
#if defined(USE_GPS)
    // waypoints are specified as latitude/longitude/altitude coordinates
    std::array<geographic_coordinate_t, MAX_WAYPOINT_COUNT> _waypoints;
#endif
};
