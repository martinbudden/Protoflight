#pragma once


#include "Autopilot.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#if defined(USE_OSD)
#include "OSD.h"
#endif
#if defined(USE_VTX)
#include "VTX.h"
#endif
#if defined(USE_GPS)
#include "GPS.h"
#endif
#include <MotorMixerBase.h>

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

#endif // USE_FLASH_KLV

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

    enum calibration_state_e { NOT_CALIBRATED = 0, CALIBRATED = 1 };
    enum { DEFAULT_PID_PROFILE = 0, PID_PROFILE_COUNT = 4 };
    enum { DEFAULT_RATE_PROFILE = 0, RATE_PROFILE_COUNT = 4 };

public:
    // NOTE: "get" functions are declared const, since they are logically const, although not physically const

    NonVolatileStorage();
    explicit NonVolatileStorage(uint32_t flashMemorySize);
    void init();
    static void toHexChars(char* charPtr, uint16_t value);

    int32_t clear();
    int32_t remove(uint16_t key);

    bool loadItem(uint16_t key, void* item, size_t length) const;
    bool loadItem(uint16_t key, uint8_t pidProfileIndex, void* item, size_t length) const;

    int32_t storeItem(uint16_t key, const void* item, size_t length, const void* defaults);
    int32_t storeItem(uint16_t key, uint8_t pidProfileIndex, const void* item, size_t length, const void* defaults);

    int32_t storeAll(const IMU_Filters& imuFilters, const FlightController& flightController, const Cockpit& cockpit, uint8_t pidProfile, uint8_t ratesProfile);

    void loadMacAddress(uint8_t* macAddress) const;
    int32_t storeMacAddress(const uint8_t* macAddress);

    uint8_t getCurrentPidProfileIndex() const { return _currentPidProfileIndex; }
    void setCurrentPidProfileIndex(uint8_t currentPidProfileIndex) { _currentPidProfileIndex = currentPidProfileIndex; }
    uint8_t getCurrentRateProfileIndex() const { return _currentRateProfileIndex; }
    void setCurrentRateProfileIndex(uint8_t currentRateProfileIndex) { _currentRateProfileIndex = currentRateProfileIndex; }

    uint8_t loadPidProfileIndex() const;
    int32_t storePidProfileIndex(uint8_t pidProfileIndex);

    uint8_t loadRateProfileIndex() const;
    int32_t storeRateProfileIndex(uint8_t rateProfileIndex);

    calibration_state_e loadAccCalibrationState() const;
    int32_t storeAccCalibrationState(calibration_state_e calibrationState);

    xyz_t loadAccOffset() const;
    int32_t storeAccOffset(const xyz_t& offset);

    calibration_state_e loadGyroCalibrationState() const;
    int32_t storeGyroCalibrationState(calibration_state_e calibrationState);

    xyz_t loadGyroOffset() const;
    int32_t storeGyroOffset(const xyz_t& offset);

    DynamicIdleController::config_t loadDynamicIdleControllerConfig(uint8_t pidProfileIndex) const;
    int32_t storeDynamicIdleControllerConfig(const DynamicIdleController::config_t& config, uint8_t pidProfileIndex);

    MotorMixerBase::mixer_config_t loadMotorMixerConfig() const;
    int32_t storeMotorMixerConfig(const MotorMixerBase::mixer_config_t& config);

    MotorMixerBase::motor_config_t loadMotorConfig() const;
    int32_t storeMotorConfig(const MotorMixerBase::motor_config_t& config);

    IMU_Filters::config_t loadIMU_FiltersConfig() const;
    int32_t storeIMU_FiltersConfig(const IMU_Filters::config_t& config);

    Cockpit::failsafe_config_t loadFailsafeConfig() const;
    int32_t storeFailsafeConfig(const Cockpit::failsafe_config_t& config);

    Features::config_t loadFeaturesConfig() const;
    int32_t storeFeaturesConfig(const Features::config_t& config);

    RX::config_t loadRX_Config() const;
    int32_t storeRX_Config(const RX::config_t& config);

    rc_modes_activation_condition_array_t loadRC_ModeActivationConditions() const;
    int32_t storeRC_ModeActivationConditions(const rc_modes_activation_condition_array_t& modeActivationConditions);

#if defined(USE_RC_ADJUSTMENTS)
    RC_Adjustments::adjustment_ranges_t loadRC_AdjustmentRanges() const;
    int32_t storeRC_AdjustmentRanges(const RC_Adjustments::adjustment_ranges_t& adjustmentRanges);
#endif
    rates_t loadRates(uint8_t rateProfileIndex) const;
    int32_t storeRates(const rates_t& rates, uint8_t rateProfileIndex);

    VehicleControllerBase::PIDF_uint16_t loadPID(uint8_t pidIndex, uint8_t pidProfileIndex) const;
    int32_t storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex, uint8_t pidProfileIndex);
    int32_t storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex) { return storePID(pid, pidIndex, _currentPidProfileIndex); }
    void resetPID(uint8_t pidIndex, uint8_t pidProfileIndex);
    void resetPID(uint8_t pidIndex) { resetPID(pidIndex, _currentPidProfileIndex); }

    FlightController::simplified_pid_settings_t loadSimplifiedPID_settings(uint8_t pidProfileIndex) const;
    int32_t storeSimplifiedPID_settings(const FlightController::simplified_pid_settings_t& settings, uint8_t pidProfileIndex);

    FlightController::filters_config_t loadFlightControllerFiltersConfig(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerFiltersConfig(const FlightController::filters_config_t& config, uint8_t pidProfileIndex);

    FlightController::flight_mode_config_t loadFlightControllerFlightModeConfig(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerFlightModeConfig(const FlightController::flight_mode_config_t& config, uint8_t pidProfileIndex);

    FlightController::tpa_config_t loadFlightControllerTPA_Config(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerTPA_Config(const FlightController::tpa_config_t& config, uint8_t pidProfileIndex);

    FlightController::anti_gravity_config_t loadFlightControllerAntiGravityConfig(uint8_t pidProfileIndex) const;
    int32_t storeFlightControllerAntiGravityConfig(const FlightController::anti_gravity_config_t& config, uint8_t pidProfileIndex);

    FlightController::crash_flip_config_t loadFlightControllerCrashFlipConfig() const;
    int32_t storeFlightControllerCrashFlipConfig(const FlightController::crash_flip_config_t& config);

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
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    DynamicNotchFilter::config_t loadDynamicNotchFilterConfig() const;
    int32_t storeDynamicNotchFilterConfig(const DynamicNotchFilter::config_t& config);
#endif
#if defined(USE_RPM_FILTERS)
    RpmFilters::config_t loadRPM_FiltersConfig() const;
    int32_t storeRPM_FiltersConfig(const RpmFilters::config_t& config);
#endif
#if defined(USE_VTX)
    VTX::config_t loadVTX_Config() const;
    int32_t storeVTX_Config(const VTX::config_t& config);
#endif
#if defined(USE_OSD)
    OSD::config_t loadOSD_Config() const;
    int32_t storeOSD_Config(const OSD::config_t& config);
    bool loadOSD_ElementsConfig(OSD_Elements::config_t & config) const;
    int32_t storeOSD_ElementsConfig(const OSD_Elements::config_t& config);
#endif
#if defined(USE_GPS)
    GPS::config_t loadGPS_Config() const;
    int32_t storeGPS_Config(const GPS::config_t& config);
#endif
#if defined(USE_ALTITUDE_HOLD)
    Autopilot::autopilot_config_t loadAutopilotConfig() const;
    int32_t storeAutopilotConfig(const Autopilot::autopilot_config_t& config);
    Autopilot::position_config_t loadAutopilotPositionConfig() const;
    int32_t storeAutopilotPositionConfig(const Autopilot::position_config_t& config);
    Autopilot::altitude_hold_config_t loadAltitudeHoldConfig() const;
    int32_t storeAltitudeHoldConfig(const Autopilot::altitude_hold_config_t& config);
#endif

private:
#if defined(USE_FLASH_KLV)
    FlashKLV _flashKLV;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    mutable Preferences _preferences {};
#endif
    uint8_t _currentPidProfileIndex {0};
    uint8_t _currentRateProfileIndex {0};
};
