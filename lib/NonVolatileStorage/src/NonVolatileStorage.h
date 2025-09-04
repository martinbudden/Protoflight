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

#include <array>
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
    struct xyz_int32_t {
        int32_t x;
        int32_t y;
        int32_t z;
    };
public:
    // NOTE: "get" functions are declared const, since they are logically const, although not physically const

    void init();
    void clear();
    void remove(const std::string& name);
    void storeAll(const AHRS& ahrs, const FlightController& flightController, const RadioController& radioController, const ReceiverBase& receiver);

    static const char* AccOffsetKey;
    bool AccOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const;
    void AccOffsetStore(int32_t x, int32_t y, int32_t z);

    static const char* GyroOffsetKey;
    bool GyroOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const;
    void GyroOffsetStore(int32_t x, int32_t y, int32_t z);

    static const char* MacAddressKey;
    void MacAddressLoad(uint8_t* macAddress) const;
    void MacAddressStore(const uint8_t* macAddress);

    static const std::array<std::string, FlightController::PID_COUNT> PID_Keys;
    PIDF::PIDF_t PID_load(uint8_t index) const;
    void PID_store(uint8_t index, const PIDF::PIDF_t& pid);

    static const char* DynamicIdleControllerConfigKey;
    DynamicIdleController::config_t DynamicIdleControllerConfigLoad() const;
    void  DynamicIdleControllerConfigStore(const DynamicIdleController::config_t& config);

    static const char* FlightControllerFiltersConfigKey;
    FlightController::filters_config_t FlightControllerFiltersConfigLoad() const;
    void FlightControllerFiltersConfigStore(const FlightController::filters_config_t& config);

    static const char* ImuFiltersConfigKey;
    IMU_Filters::config_t ImuFiltersConfigLoad() const;
    void ImuFiltersConfigStore(const IMU_Filters::config_t& config);

    static const char* RadioControllerRatesKey;
    RadioController::rates_t RadioControllerRatesLoad() const;
    void RadioControllerRatesStore(const RadioController::rates_t& rates);
private:
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    mutable Preferences _preferences;
#endif
};
