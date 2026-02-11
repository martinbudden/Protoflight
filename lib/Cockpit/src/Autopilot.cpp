#include "Autopilot.h"
#include <AHRS.h>
#include <AHRS_MessageQueue.h>
#include <AltitudeMessageQueue.h>
#include <BarometerBase.h>


void Autopilot::setAutopilotConfig(const autopilot_config_t& autopilotConfig)
{
    _autopilotConfig = autopilotConfig;
    _altitude.hoverThrottle = static_cast<float>(_autopilotConfig.throttle_hover_pwm - 1000) * 0.001F;

    static constexpr float ALTITUDE_P_SCALE  = 0.01F;
    static constexpr float ALTITUDE_I_SCALE  = 0.003F;
    static constexpr float ALTITUDE_D_SCALE  = 0.01F;
    static constexpr float ALTITUDE_S_SCALE  = 0.01F;
    static constexpr float ALTITUDE_F_SCALE  = 0.01F;
    const PIDF::PIDF_t altitudePID = {
        static_cast<float>(_autopilotConfig.altitudePID.kp) * ALTITUDE_P_SCALE,
        static_cast<float>(_autopilotConfig.altitudePID.ki) * ALTITUDE_I_SCALE,
        static_cast<float>(_autopilotConfig.altitudePID.kd) * ALTITUDE_D_SCALE,
        static_cast<float>(_autopilotConfig.altitudePID.ks) * ALTITUDE_S_SCALE,
        static_cast<float>(_autopilotConfig.altitudePID.kk) * ALTITUDE_F_SCALE,
    };
    _altitude.pid.setPID(altitudePID);
    _altitude.pid.setSetpoint(std::numeric_limits<float>::lowest());

    static constexpr float POSITION_P_SCALE  = 0.0012F;
    static constexpr float POSITION_I_SCALE  = 0.0001F;
    static constexpr float POSITION_D_SCALE  = 0.0015F;
    static constexpr float POSITION_S_SCALE  = 0.0015F;
    static constexpr float POSITION_F_SCALE  = 0.0008F;
    const PIDF::PIDF_t positionPID = {
        static_cast<float>(_autopilotConfig.positionPID.kp) * POSITION_P_SCALE,
        static_cast<float>(_autopilotConfig.positionPID.ki) * POSITION_I_SCALE,
        static_cast<float>(_autopilotConfig.positionPID.kd) * POSITION_D_SCALE,
        static_cast<float>(_autopilotConfig.positionPID.ks) * POSITION_S_SCALE,
        static_cast<float>(_autopilotConfig.positionPID.kk) * POSITION_F_SCALE,
    };
    _altitude.pid.setPID(positionPID);
}

#if defined(USE_GPS_RESCUE)
void Autopilot::setGPS_RescueConfig(const gps_rescue_config_t& gpsRescueConfig)
{
    _gpsRescueConfig = gpsRescueConfig;
}
#endif

void Autopilot::setPositionConfig(const position_config_t& positionConfig)
{
    _positionConfig = positionConfig;
}

void Autopilot::setAltitudeHoldConfig(const altitude_hold_config_t& altitudeHoldConfig)
{
    _altitudeHoldConfig = altitudeHoldConfig;
}

bool Autopilot::isAltitudeHoldSetpointSet() const
{
    return (_altitude.pid.getSetpoint() == std::numeric_limits<float>::lowest()) ? false : true;
}

bool Autopilot::setAltitudeHoldSetpoint()
{
    if (_altitudeMessageQueue == nullptr) {
        // no barometer, so cannot go into altitude hold mode
        return false;
    }

    altitude_data_t altitudeData {};
    _altitudeMessageQueue->PEEK_ALTITUDE_DATA(altitudeData);
    _altitude.pid.setSetpoint(altitudeData.altitudeMeters);
    return true;
}

float Autopilot::calculateThrottleForAltitudeHold(const receiver_controls_t& controls)
{
    if (_altitudeMessageQueue == nullptr) {
        return controls.throttle;
    }

    altitude_data_t altitudeData {};
    _altitudeMessageQueue->PEEK_ALTITUDE_DATA(altitudeData);

    //const ahrs_data_t queueItem = _ahrsMessageQueue.getQueueItem();
    //const Quaternion orientation = queueItem.orientation;
    const float cosTiltAngle = 1.0F; //!!TODO:get from AHRS
    const float deltaT = 0.001F; //!!TODO:set in startup

    const float altitudeDeltaFilteredMeters = _altitude.dTermLPF.filter(altitudeData.altitudeMeters - _altitude.pid.getPreviousMeasurement());
    float throttle = _altitude.pid.updateDelta(altitudeData.altitudeMeters, altitudeDeltaFilteredMeters, deltaT);

    throttle += _altitude.hoverThrottle;

    // 1.0 when level, 1.3 at 40 degrees, 1.56 at 50 deg, maximum 2.0 at 60 degrees or higher,
    // the default limit for Angle Mode is 60 degrees
    const float tiltMultiplier = 1.0F / std::fmaxf(cosTiltAngle, 0.5F);

    throttle *= tiltMultiplier;

    return throttle;
}

FlightController::controls_t Autopilot::calculateFlightControls(const receiver_controls_t& controls, uint32_t flightModeModeFlags)
{
    (void)flightModeModeFlags;

    const float throttle = calculateThrottleForAltitudeHold(controls);

    const FlightController::controls_t flightControls = {
        .tickCount = 0, //tickCount,
        .throttleStick = throttle,
        .rollStickDPS = 0,
        .pitchStickDPS = 0,
        .yawStickDPS = 0,
        .rollStickDegrees = 0,
        .pitchStickDegrees = 0,
        .controlMode = FlightController::CONTROL_MODE_ANGLE
    };

    return flightControls;
}
