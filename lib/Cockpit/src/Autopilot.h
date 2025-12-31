#pragma once

#include "FlightController.h"

#include <CockpitBase.h>
#include <Filters.h>
#if defined(USE_GPS)
#include <GeographicCoordinate.h>
#endif
#include <PIDF.h>


class AHRS_MessageQueue;
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
    // flight mode flags
    enum log2_flight_mode_flag_e {
        LOG2_ANGLE_MODE         = 0,
        LOG2_HORIZON_MODE       = 1,
        LOG2_MAG_MODE           = 2,
        LOG2_ALTITUDE_HOLD_MODE = 3,
        LOG2_GPS_HOME_MODE      = 4,
        LOG2_POSITION_HOLD_MODE = 5,
        LOG2_HEADFREE_MODE      = 6,
        LOG2_CHIRP_MODE         = 7,
        LOG2_PASSTHRU_MODE      = 8,
        LOG2_RANGEFINDER_MODE   = 9,
        LOG2_FAILSAFE_MODE      = 10,
        LOG2_GPS_RESCUE_MODE    = 11
    };
    static constexpr uint32_t ANGLE_MODE      = 1U << LOG2_ANGLE_MODE;
    static constexpr uint32_t HORIZON_MODE    = 1U << LOG2_HORIZON_MODE;
    static constexpr uint32_t MAG_MODE        = 1U << LOG2_MAG_MODE;
    static constexpr uint32_t ALTITUDE_HOLD_MODE = 1U << LOG2_ALTITUDE_HOLD_MODE;
    static constexpr uint32_t GPS_HOME_MODE   = 1U << LOG2_GPS_HOME_MODE;
    static constexpr uint32_t POSITION_HOLD_MODE = 1U << LOG2_POSITION_HOLD_MODE;
    static constexpr uint32_t HEADFREE_MODE   = 1U << LOG2_HEADFREE_MODE;
    static constexpr uint32_t CHIRP_MODE      = 1U << LOG2_CHIRP_MODE;
    static constexpr uint32_t PASSTHRU_MODE   = 1U << LOG2_PASSTHRU_MODE;
    static constexpr uint32_t RANGEFINDER_MODE= 1U << LOG2_RANGEFINDER_MODE;
    static constexpr uint32_t FAILSAFE_MODE   = 1U << LOG2_FAILSAFE_MODE;
    static constexpr uint32_t GPS_RESCUE_MODE = 1U << LOG2_GPS_RESCUE_MODE;
    struct PIDF_uint16_t {
        uint16_t kp;
        uint16_t ki;
        uint16_t kd;
        uint16_t ks;
        uint16_t kk;
    };
    struct autopilot_config_t {
        PIDF_uint16_t altitudePID;
        PIDF_uint16_t positionPID;
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
