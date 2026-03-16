#include "autopilot.h"
#include "barometer_base.h"

#include <ahrs.h>
#include <ahrs_message_queue.h>
#include <altitude_message_queue.h>


void Autopilot::set_autopilot_config(const autopilot_config_t& autopilotConfig)
{
    _autopilotConfig = autopilotConfig;
    _altitude.hoverThrottle = static_cast<float>(_autopilotConfig.throttle_hover_pwm - 1000) * 0.001F;

    static constexpr float ALTITUDE_P_SCALE  = 0.01F;
    static constexpr float ALTITUDE_I_SCALE  = 0.003F;
    static constexpr float ALTITUDE_D_SCALE  = 0.01F;
    static constexpr float ALTITUDE_S_SCALE  = 0.01F;
    static constexpr float ALTITUDE_F_SCALE  = 0.01F;
    const pid_constants_t altitude_pid = {
        static_cast<float>(_autopilotConfig.altitude_pid.kp) * ALTITUDE_P_SCALE,
        static_cast<float>(_autopilotConfig.altitude_pid.ki) * ALTITUDE_I_SCALE,
        static_cast<float>(_autopilotConfig.altitude_pid.kd) * ALTITUDE_D_SCALE,
        static_cast<float>(_autopilotConfig.altitude_pid.ks) * ALTITUDE_S_SCALE,
        static_cast<float>(_autopilotConfig.altitude_pid.kk) * ALTITUDE_F_SCALE,
    };
    _altitude.pid.set_pid(altitude_pid);
    _altitude.pid.set_setpoint(std::numeric_limits<float>::lowest());

    static constexpr float POSITION_P_SCALE  = 0.0012F;
    static constexpr float POSITION_I_SCALE  = 0.0001F;
    static constexpr float POSITION_D_SCALE  = 0.0015F;
    static constexpr float POSITION_S_SCALE  = 0.0015F;
    static constexpr float POSITION_F_SCALE  = 0.0008F;
    const pid_constants_t position_pid = {
        static_cast<float>(_autopilotConfig.position_pid.kp) * POSITION_P_SCALE,
        static_cast<float>(_autopilotConfig.position_pid.ki) * POSITION_I_SCALE,
        static_cast<float>(_autopilotConfig.position_pid.kd) * POSITION_D_SCALE,
        static_cast<float>(_autopilotConfig.position_pid.ks) * POSITION_S_SCALE,
        static_cast<float>(_autopilotConfig.position_pid.kk) * POSITION_F_SCALE,
    };
    _altitude.pid.set_pid(position_pid);
}

#if defined(USE_GPS_RESCUE)
void Autopilot::setGPS_RescueConfig(const gps_rescue_config_t& gpsRescueConfig)
{
    _gpsRescueConfig = gpsRescueConfig;
}
#endif

void Autopilot::setPositionHoldConfig(const position_hold_config_t& positionHoldConfig)
{
    _positionHoldConfig = positionHoldConfig;
}

void Autopilot::set_altitude_hold_config(const altitude_hold_config_t& altitudeHoldConfig)
{
    _altitudeHoldConfig = altitudeHoldConfig;
}

bool Autopilot::isAltitudeHoldSetpointSet() const
{
    return (_altitude.pid.get_setpoint() == std::numeric_limits<float>::lowest()) ? false : true;
}

bool Autopilot::setAltitudeHoldSetpoint()
{
    if (_altitudeMessageQueue == nullptr) {
        // no barometer, so cannot go into altitude hold mode
        return false;
    }

    altitude_data_t altitude_data {};
    _altitudeMessageQueue->PEEK_ALTITUDE_DATA(altitude_data);
    _altitude.pid.set_setpoint(altitude_data.altitude_meters);
    return true;
}

float Autopilot::calculateThrottleForAltitudeHold(const receiver_controls_t& controls)
{
    if (_altitudeMessageQueue == nullptr) {
        return controls.throttle;
    }

    altitude_data_t altitude_data {};
    _altitudeMessageQueue->PEEK_ALTITUDE_DATA(altitude_data);

    //const ahrs_data_t queueItem = _ahrs_message_queue.getQueueItem();
    //const Quaternion orientation = queueItem.orientation;
    const float cosTiltAngle = 1.0F; //!!TODO:get from AHRS
    const float delta_t = 0.001F; //!!TODO:set in startup

    const float altitudeDeltaFilteredMeters = _altitude.dTermLPF.filter(altitude_data.altitude_meters - _altitude.pid.get_previous_measurement());
    float throttle = _altitude.pid.update_delta(altitude_data.altitude_meters, altitudeDeltaFilteredMeters, delta_t);

    throttle += _altitude.hoverThrottle;

    // 1.0 when level, 1.3 at 40 degrees, 1.56 at 50 deg, maximum 2.0 at 60 degrees or higher,
    // the default limit for Angle Mode is 60 degrees
    const float tiltMultiplier = 1.0F / std::fmaxf(cosTiltAngle, 0.5F);

    throttle *= tiltMultiplier;

    return throttle;
}

fc_controls_t Autopilot::calculateFlightControls(const receiver_controls_t& controls, uint32_t flightMode_mode_flags)
{
    (void)flightMode_mode_flags;

    const float throttle = calculateThrottleForAltitudeHold(controls);

    const fc_controls_t flightControls = {
        .tick_count = 0, //tick_count,
        .throttle_stick = throttle,
        .roll_stick_dps = 0,
        .pitch_stick_dps = 0,
        .yaw_stick_dps = 0,
        .roll_stick_degrees = 0,
        .pitch_stick_degrees = 0,
        .control_mode = FC_CONTROL_MODE_ANGLE
    };

    return flightControls;
}
