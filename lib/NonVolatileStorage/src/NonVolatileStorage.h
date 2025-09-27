#pragma once

#include "Defaults.h"


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

#include <cstdint>
#include <string>


/*!
*/
class NonVolatileStorage {
public:
    enum {
        OK = 0, OK_IS_DEFAULT,
        ERROR_FLASH_FULL = -1, ERROR_NOT_FOUND = -2, ERROR_NOT_WRITTEN = -3, ERROR_INVALID_PROFILE
    };

    enum { READ_WRITE=false, READ_ONLY=true };
    enum { MAC_ADDRESS_LEN = 6 };

    enum { DEFAULT_PID_PROFILE = 0, PID_PROFILE_COUNT = 4 };
    enum { DEFAULT_RATE_PROFILE = 0, RATE_PROFILE_COUNT = 4 };

    struct xyz_int32_t {
        int32_t x;
        int32_t y;
        int32_t z;
    };
public:
    // NOTE: "get" functions are declared const, since they are logically const, although not physically const

    NonVolatileStorage();
    explicit NonVolatileStorage(uint32_t flashMemorySize);
    void init();
    static size_t min(size_t a, uint16_t b) { return a > b ? b : a; }

    uint8_t getCurrentPidProfileIndex() const { return _currentPidProfileIndex; }
    void setCurrentPidProfileIndex(uint8_t currentPidProfileIndex) { _currentPidProfileIndex = currentPidProfileIndex; }

    int32_t clear();
#if defined(USE_FLASH_KLV)
    int32_t remove(uint16_t key);
#else
    int32_t remove(const std::string& name);
#endif
    int32_t storeAll(const AHRS& ahrs, const FlightController& flightController, const RadioController& radioController, const ReceiverBase& receiver, uint8_t pidProfile, uint8_t ratesProfile);

    bool AccOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const;
    int32_t AccOffsetStore(int32_t x, int32_t y, int32_t z);

    bool GyroOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const;
    int32_t GyroOffsetStore(int32_t x, int32_t y, int32_t z);

    void MacAddressLoad(uint8_t* macAddress) const;
    int32_t MacAddressStore(const uint8_t* macAddress);

    uint8_t PidProfileIndexLoad() const;
    int32_t PidProfileIndexStore(uint8_t pidProfileIndex);

    uint8_t RateProfileIndexLoad() const;
    int32_t RateProfileIndexStore(uint8_t rateProfileIndex);

    VehicleControllerBase::PIDF_uint16_t PID_load(uint8_t pidIndex, uint8_t pidProfileIndex) const;
    int32_t PID_store(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex, uint8_t pidProfileIndex);
    int32_t PID_store(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex) { return PID_store(pid, pidIndex, _currentPidProfileIndex); }
    void PID_reset(uint8_t index);

    DynamicIdleController::config_t DynamicIdleControllerConfigLoad(uint8_t pidProfileIndex) const;
    int32_t DynamicIdleControllerConfigStore(const DynamicIdleController::config_t& config, uint8_t pidProfileIndex);

    FlightController::filters_config_t FlightControllerFiltersConfigLoad(uint8_t pidProfileIndex) const;
    int32_t FlightControllerFiltersConfigStore(const FlightController::filters_config_t& config, uint8_t pidProfileIndex);

    FlightController::anti_gravity_config_t FlightControllerAntiGravityConfigLoad(uint8_t pidProfileIndex) const;
    int32_t FlightControllerAntiGravityConfigStore(const FlightController::anti_gravity_config_t& config, uint8_t pidProfileIndex);

    FlightController::d_max_config_t FlightControllerDMaxConfigLoad(uint8_t pidProfileIndex) const;
    int32_t FlightControllerDMaxConfigStore(const FlightController::d_max_config_t& config, uint8_t pidProfileIndex);

    IMU_Filters::config_t ImuFiltersConfigLoad() const;
    int32_t ImuFiltersConfigStore(const IMU_Filters::config_t& config);

    RadioController::failsafe_t RadioControllerFailsafeLoad();
    int32_t RadioControllerFailsafeStore(const RadioController::failsafe_t& failsafe);

    RadioController::rates_t RadioControllerRatesLoad(uint8_t rateProfileIndex) const;
    int32_t RadioControllerRatesStore(const RadioController::rates_t& rates, uint8_t rateProfileIndex);

private:
#if defined(USE_FLASH_KLV)
    FlashKLV _flashKLV;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    mutable Preferences _preferences {};
#endif
    uint8_t _currentPidProfileIndex {0};
};
