#pragma once

#include "Defaults.h"

#define USE_FLASH_KLV

#if defined(USE_FLASH_KLV)

#include <FlashKLV.h>

#else

// ESP32 and/or ARDUINO_ARCH_ESP32 are defined by Arduino in platform.txt
#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#if !defined(USE_ARDUINO_ESP32_PREFERENCES)
#define USE_ARDUINO_ESP32_PREFERENCES
#endif
#endif

#if defined(USE_ARDUINO_ESP32_PREFERENCES)
#include <Preferences.h>
#endif

#endif

#include <array>
#include <cfloat>
#include <cstdint>
#include <string>


/*!
*/
class NonVolatileStorage {
public:
    enum {
        OK = 0, OK_IS_DEFAULT,
        ERROR_FLASH_FULL = -1, ERROR_NOT_FOUND = -2, ERROR_NOT_WRITTEN = -3
    };

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

    NonVolatileStorage();
    NonVolatileStorage(uint32_t flashMemorySize);
    void init();
    static size_t min(size_t a, uint16_t b) { return a > b ? b : a; }
    int32_t clear();
#if defined(USE_FLASH_KLV)
    int32_t remove(uint16_t key);
#else
    int32_t remove(const std::string& name);
#endif
    int32_t storeAll(const AHRS& ahrs, const FlightController& flightController, const RadioController& radioController, const ReceiverBase& receiver);

    bool AccOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const;
    int32_t AccOffsetStore(int32_t x, int32_t y, int32_t z);

    bool GyroOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const;
    int32_t GyroOffsetStore(int32_t x, int32_t y, int32_t z);

    void MacAddressLoad(uint8_t* macAddress) const;
    int32_t MacAddressStore(const uint8_t* macAddress);

    PIDF::PIDF_t PID_load(uint8_t index) const;
    int32_t PID_store(uint8_t index, const PIDF::PIDF_t& pid);

    DynamicIdleController::config_t DynamicIdleControllerConfigLoad() const;
    int32_t DynamicIdleControllerConfigStore(const DynamicIdleController::config_t& config);

    FlightController::filters_config_t FlightControllerFiltersConfigLoad() const;
    int32_t FlightControllerFiltersConfigStore(const FlightController::filters_config_t& config);

    IMU_Filters::config_t ImuFiltersConfigLoad() const;
    int32_t ImuFiltersConfigStore(const IMU_Filters::config_t& config);

    RadioController::failsafe_t RadioControllerFailsafeLoad();
    int32_t RadioControllerFailsafeStore(const RadioController::failsafe_t& failsafe);

    RadioController::rates_t RadioControllerRatesLoad() const;
    int32_t RadioControllerRatesStore(const RadioController::rates_t& rates);
private:
#if defined(USE_FLASH_KLV)
    FlashKLV _flashKLV;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    mutable Preferences _preferences;
#endif
};
