#pragma once

#include <AHRS_MessageQueue.h>
#include <FlightController.h>
#include <Filters.h>
#include <RadioControllerBase.h>
#include <PIDF.h>

#include <array>
#include <cstdint>

class BarometerBase;


class Autopilot {
public:
    virtual ~Autopilot() = default;
    Autopilot(const AHRS_MessageQueue& messageQueue) : _messageQueue(messageQueue) {}
    Autopilot(const AHRS_MessageQueue& messageQueue, BarometerBase& barometer) : _messageQueue(messageQueue), _barometer(&barometer) {}
private:
    // Autopilot is not copyable or moveable
    Autopilot(const Autopilot&) = delete;
    Autopilot& operator=(const Autopilot&) = delete;
    Autopilot(Autopilot&&) = delete;
    Autopilot& operator=(Autopilot&&) = delete;
public:
    enum earth_frame_axis_e { LONGITUDE, LATITUDE, EARTH_FRAME_AXIS_COUNT };
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
        uint16_t throttleHover;
        uint16_t throttleMin;
        uint16_t throttleMax;
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

    bool setAltitudeHoldSetpoint(); //!< use the current altitude to set the setpoint for altitude hold
    float calculateThrottleForAltitudeHold(const RadioControllerBase::controls_t& controls);
    FlightController::controls_t calculateFlightControls(const RadioControllerBase::controls_t& controls, uint32_t flightModeModeFlags);
private:
    const AHRS_MessageQueue& _messageQueue;
    BarometerBase* _barometer {nullptr};
    altitude_t _altitude {};
    std::array<earth_frame_t, EARTH_FRAME_AXIS_COUNT> _earthFrames {};
    autopilot_config_t _autopilotConfig;
    position_config_t _positionConfig;
    altitude_hold_config_t _altitudeHoldConfig;
};
