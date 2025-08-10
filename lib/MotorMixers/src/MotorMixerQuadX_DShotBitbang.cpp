#include "DynamicIdleController.h"
#include "MotorMixerQuadX_DShotBitbang.h"

#include <Debug.h>
#include <Filters.h>
#include <RPM_Filters.h>
#include <algorithm>
#include <cmath>


MotorMixerQuadX_DShotBitbang::MotorMixerQuadX_DShotBitbang(Debug& debug, const pins_t& pins, RPM_Filters& rpmFilters, DynamicIdleController& dynamicIdleController) :
    MotorMixerQuadX_Base(debug),
    _rpmFilters(rpmFilters),
    _dynamicIdleController(dynamicIdleController)
{
}

float MotorMixerQuadX_DShotBitbang::calculateSlowestMotorHz() const
{
    const float slowestMotorHz = 0.0F;
    return slowestMotorHz;
}

DynamicIdleController* MotorMixerQuadX_DShotBitbang::getDynamicIdleController() const
{
    return &_dynamicIdleController;
}

void MotorMixerQuadX_DShotBitbang::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void)tickCount;
    (void)deltaT;
    (void)commands;
}
