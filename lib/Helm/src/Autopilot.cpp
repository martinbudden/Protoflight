#include "Autopilot.h"
#include <AHRS.h>
#include <BarometerBase.h>


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

float Autopilot::altitudeHoldCalculateThrottle(float throttle)
{
    if (_barometer == nullptr) {
        return throttle;
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
    throttle = _altitude.pid.updateDeltaITerm(altitudeMeters, altitudeDeltaFilteredMeters*dBoost, altitudeErrorMeters*iTermRelax, deltaT);
#else
    throttle = _altitude.pid.updateDelta(altitudeMeters, altitudeDeltaFilteredMeters*dBoost, deltaT);
#endif

    throttle += _altitude.hoverThrottle;

    // 1.0 when level, 1.3 at 40 degrees, 1.56 at 50 deg, maximum 2.0 at 60 degrees or higher,
    // the default limit for Angle Mode is 60 degrees
    const float tiltMultiplier = 1.0F / std::fmaxf(cosTiltAngle, 0.5F);

    throttle *= tiltMultiplier;

    return throttle;
}
