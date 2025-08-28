#pragma once

#include "Defaults.h"

// ESP32 and/or ARDUINO_ARCH_ESP32 are defined by Arduino in platform.txt
#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#if !defined(USE_ARDUINO_ESP32_PREFERENCES)
#define USE_ARDUINO_ESP32_PREFERENCES
#endif
#endif

#if defined(USE_ARDUINO_ESP32_PREFERENCES)
#include <Preferences.h>
#endif

#include <cfloat>
#include <cstdint>
#include <string>


/*!
Just dummy up NVS at the moment by loading defaults.
*/
class NonVolatileStorage {
public:
    enum { READ_WRITE=false, READ_ONLY=true };
    enum { MAC_ADDRESS_LEN = 6 };
    static constexpr float NOT_SET = FLT_MAX;
public:
    // NOTE: "get" functions are declared const, since they are logically const, although not physically const

    void clear();
    bool isSetPID() const;

    PIDF::PIDF_t getPID(const std::string& name) const;
    void putPID(const std::string& name, const PIDF::PIDF_t& pid);

    float getFloat(const std::string& name) const;
    void putFloat(const std::string& name, float value);

    void removeAccOffset();
    bool getAccOffset(int32_t& x, int32_t& y, int32_t& z) const;
    void putAccOffset(int32_t x, int32_t y, int32_t z);

    void removeGyroOffset();
    bool getGyroOffset(int32_t& x, int32_t& y, int32_t& z) const;
    void putGyroOffset(int32_t x, int32_t y, int32_t z);

    void getMacAddress(uint8_t* macAddress, const std::string& name) const;
    void putMacAddress(const std::string& name, const uint8_t* macAddress);

    IMU_Filters::config_t loadImuFiltersConfig() const;
    void storeImuFiltersConfig(const IMU_Filters::config_t& config);

    DynamicIdleController::config_t loadDynamicIdleControllerConfig() const;
    void  storeDynamicIdleControllerConfig(const DynamicIdleController::config_t& config);

    RadioController::rates_t loadRadioControllerRates() const;
    void storeRadioControllerRates(const RadioController::rates_t& config);

    void storeAll() {} // placeholder
private:
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    mutable Preferences _preferences;
#endif
};
