#include "Autopilot.h"
#include <AHRS.h>
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

void Autopilot::setPositionConfig(const position_config_t& positionConfig)
{
    _positionConfig = positionConfig;
}

void Autopilot::setAltitudeHoldConfig(const altitude_hold_config_t& altitudeHoldConfig)
{
    _altitudeHoldConfig = altitudeHoldConfig;
}

bool Autopilot::setAltitudeHoldSetpoint()
{
    if (_barometer == nullptr) {
        // no barometer, so cannot go into altitude hold mode
        return false;
    }

    const float altitudeMeters = _barometer->readAltitudeMeters();
    _altitude.pid.setSetpoint(altitudeMeters);
    return true;
}

float Autopilot::calculateThrottleForAltitudeHold(const CockpitBase::controls_t& controls)
{
    if (_barometer == nullptr) {
        return controls.throttleStick;
    }

    //const AHRS::ahrs_data_t queueItem = _messageQueue.getQueueItem();
    //const Quaternion orientation = queueItem.orientation;

    const float altitudeMeters = _barometer->readAltitudeMeters();
    const float cosTiltAngle = 1.0F; //!!TODO:get from AHRS
    const float deltaT = 0.001F; //!!TODO:set in startup

    const float dBoost = 1.0F;
    const float altitudeDeltaFilteredMeters = _altitude.dTermLPF.filter(altitudeMeters - _altitude.pid.getPreviousMeasurement());

#if defined(USE_ALTITUDE_HOLD_ITERM_RELAX)
    const float altitudeErrorMeters = _altitude.pid.getSetpoint() - altitudeMeters;
    const float iTermRelax = (std::fabs(altitudeErrorMeters) < 2.0F) ? 1.0F : 0.1F;
    float throttle = _altitude.pid.updateDeltaITerm(altitudeMeters, altitudeDeltaFilteredMeters*dBoost, altitudeErrorMeters*iTermRelax, deltaT);
#else
    float throttle = _altitude.pid.updateDelta(altitudeMeters, altitudeDeltaFilteredMeters*dBoost, deltaT);
#endif

    throttle += _altitude.hoverThrottle;

    // 1.0 when level, 1.3 at 40 degrees, 1.56 at 50 deg, maximum 2.0 at 60 degrees or higher,
    // the default limit for Angle Mode is 60 degrees
    const float tiltMultiplier = 1.0F / std::fmaxf(cosTiltAngle, 0.5F);

    throttle *= tiltMultiplier;

    return throttle;
}

FlightController::controls_t Autopilot::calculateFlightControls(const CockpitBase::controls_t& controls, uint32_t flightModeModeFlags)
{
    (void)flightModeModeFlags;

    const float throttle = calculateThrottleForAltitudeHold(controls);

    const FlightController::controls_t flightControls = {
        .tickCount = controls.tickCount,
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
