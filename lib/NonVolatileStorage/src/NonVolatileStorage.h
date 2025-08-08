#pragma once

#include "Defaults.h"

/*!
Just dummy up NVS at the moment by loading defaults.
*/
class NonVolatileStorage {
public:
    IMU_Filters::config_t loadImuFiltersConfig() const { return DEFAULTS::imuFiltersConfig; }
    DynamicIdleController::config_t loadDynamicIdleControllerConfig() const { return DEFAULTS::dynamicIdleControllerConfig; }
    RadioController::rates_t loadRadioControllerRates() const { return DEFAULTS::radioControllerRates; }

    void storeAll() {} // placeholder
};
