#include "Autopilot.h"
#include <AHRS.h>


void Autopilot::setAutopilotConfig(const autopilot_config_t& autopilotConfig)
{
    _autopilotConfig = autopilotConfig;
    _altitude.hoverThrottle = (_autopilotConfig.throttleHover - 1000) * 0.001F;
}

void Autopilot::setPositionConfig(const position_config_t& positionConfig)
{
    _positionConfig = positionConfig;
}

void Autopilot::setAltitudeHoldConfig(const altitude_hold_config_t& altitudeHoldConfig)
{
    _altitudeHoldConfig = altitudeHoldConfig;
}

void Autopilot::setAltitudeHoldSetpoint()
{ 
    const float altitudeMeters = 0.0F;//!!TODO: get from barometer
    _altitude.pid.setSetpoint(altitudeMeters);
}

float Autopilot::altitudeHoldCalculateThrottle()
{
    //const AHRS::ahrs_data_t queueItem = _messageQueue.getQueueItem();
    //const Quaternion orientation = queueItem.orientation;

    const float altitudeMeters = 0.0F; // !!TODO:get from barometer
    const float cosTiltAngle = 1.0F; //!!TODO:get from AHRS
    const float deltaT = 0.001F; //!!TODO:set in startup

    const float altitudeDeltaFilteredMeters = _altitude.dTermLPF.filter(altitudeMeters - _altitude.pid.getPreviousMeasurement());
    const float altitudeErrorMeters = _altitude.pid.getSetpoint() - altitudeMeters;
    const float iTermRelax = (std::fabs(altitudeErrorMeters) < 2.0F) ? 1.0F : 0.1F;
    const float dBoost = 1.0F;

    float throttle = _altitude.pid.updateDeltaITerm(altitudeMeters, altitudeDeltaFilteredMeters*dBoost, altitudeErrorMeters*iTermRelax, deltaT);

    //const float hoverThrottle = 0.275F;
    throttle += _altitude.hoverThrottle;

    // 1.0 when level, 1.3 at 40 degrees, 1.56 at 50 deg, maximum 2.0 at 60 degrees or higher,
    // the default limit for Angle Mode is 60 degrees
    const float tiltMultiplier = 1.0F / std::fmaxf(cosTiltAngle, 0.5F);

    throttle *= tiltMultiplier;

    return throttle;
}
