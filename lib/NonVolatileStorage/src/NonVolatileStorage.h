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
    static void toHexChars(char* charPtr, uint16_t value);

    uint8_t getCurrentPidProfileIndex() const { return _currentPidProfileIndex; }
    void setCurrentPidProfileIndex(uint8_t currentPidProfileIndex) { _currentPidProfileIndex = currentPidProfileIndex; }

    int32_t clear();
    int32_t remove(uint16_t key);

    int32_t storeAll(const FlightController& flightController, const RadioController& radioController, uint8_t pidProfile, uint8_t ratesProfile);

    bool loadAccOffset(int32_t& x, int32_t& y, int32_t& z) const;
    int32_t storeAccOffset(int32_t x, int32_t y, int32_t z);

    bool loadGyroOffset(int32_t& x, int32_t& y, int32_t& z) const;
    int32_t storeGyroOffset(int32_t x, int32_t y, int32_t z);

    void loadMacAddress(uint8_t* macAddress) const;
    int32_t storeMacAddress(const uint8_t* macAddress);

    uint8_t loadPidProfileIndex() const;
    int32_t storePidProfileIndex(uint8_t pidProfileIndex);

    uint8_t loadRateProfileIndex() const;
    int32_t storeRateProfileIndex(uint8_t rateProfileIndex);

    VehicleControllerBase::PIDF_uint16_t loadPID(uint8_t pidIndex, uint8_t pidProfileIndex) const;
    int32_t storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex, uint8_t pidProfileIndex);
    int32_t storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex) { return storePID(pid, pidIndex, _currentPidProfileIndex); }
    void resetPID(uint8_t pidIndex, uint8_t pidProfileIndex);
    void resetPID(uint8_t pidIndex) { resetPID(pidIndex, _currentPidProfileIndex); }

    bool loadItem(uint16_t key, void* item, size_t length) const;
    bool loadItem(uint16_t key, uint8_t pidProfileIndex, void* item, size_t length) const;
    int32_t storeItem(uint16_t key, const void* item, size_t length, const void* defaults);
    int32_t storeItem(uint16_t key, uint8_t pidProfileIndex, const void* item, size_t length, const void* defaults);

    DynamicIdleController::config_t loadDynamicIdleControllerConfig(uint8_t pidProfileIndex) const;
    int32_t storeDynamicIdleControllerConfig(const DynamicIdleController::config_t& config, uint8_t pidProfileIndex);

    FlightController::filters_config_t loadFlightControllerFiltersConfig(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerFiltersConfig(const FlightController::filters_config_t& config, uint8_t pidProfileIndex);

    FlightController::tpa_config_t loadFlightControllerTPA_Config(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerTPA_Config(const FlightController::tpa_config_t& config, uint8_t pidProfileIndex);

    FlightController::anti_gravity_config_t loadFlightControllerAntiGravityConfig(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerAntiGravityConfig(const FlightController::anti_gravity_config_t& config, uint8_t pidProfileIndex);

#if defined(USE_D_MAX)
    FlightController::d_max_config_t loadFlightControllerDMaxConfig(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerDMaxConfig(const FlightController::d_max_config_t& config, uint8_t pidProfileIndex);
#endif
#if defined(USE_ITERM_RELAX)
    FlightController::iterm_relax_config_t loadFlightControllerITermRelaxConfig(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerITermRelaxConfig(const FlightController::iterm_relax_config_t& config, uint8_t pidProfileIndex);
#endif
#if defined(USE_YAW_SPIN_RECOVERY)
    FlightController::yaw_spin_recovery_config_t loadFlightControllerYawSpinRecoveryConfig(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerYawSpinRecoveryConfig(const FlightController::yaw_spin_recovery_config_t& config, uint8_t pidProfileIndex);
#endif
#if defined(USE_CRASH_RECOVERY)
    FlightController::crash_recovery_config_t loadFlightControllerCrashRecoveryConfig(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerCrashRecoveryConfig(const FlightController::crash_recovery_config_t& config, uint8_t pidProfileIndex);
#endif

    IMU_Filters::config_t loadIMU_FiltersConfig() const;
    int32_t storeIMU_FiltersConfig(const IMU_Filters::config_t& config);

#if defined(USE_RPM_FILTERS)
    RPM_Filters::config_t loadRPM_FiltersConfig() const;
    int32_t storeRPM_FiltersConfig(const RPM_Filters::config_t& config);
#endif

    RadioController::failsafe_t loadRadioControllerFailsafe();
    int32_t storeRadioControllerFailsafe(const RadioController::failsafe_t& failsafe);

    RadioController::rates_t loadRadioControllerRates(uint8_t rateProfileIndex) const;
    int32_t storeRadioControllerRates(const RadioController::rates_t& rates, uint8_t rateProfileIndex);

private:
#if defined(USE_FLASH_KLV)
    FlashKLV _flashKLV;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    mutable Preferences _preferences {};
#endif
    uint8_t _currentPidProfileIndex {0};
};
