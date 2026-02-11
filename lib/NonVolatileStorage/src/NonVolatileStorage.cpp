#include "Defaults.h"
#if defined(USE_CMS)
#include "CMS.h"
#endif
#include "NonVolatileStorage.h"
#include <AHRS.h>
#include <cstring>


/*
Flash KLV keys:

Keys 0x01-0x3F (1-63): the key and the length are stored as 8-bit values and so there is an
overhead of 2 bytes per record stored.

Keys 0x0100-0x1FFF (256-8191): the key is stored as a 16-bit value and the length is stored as a 8-bit value and so there is an
overhead of 3 bytes per record stored.

Keys 0x2000-0x3FFD (8192-16361): the key and the length are stored as 16-bit values and so there is an
overhead of 4 bytes per record stored.

Other keys are invalid and may not be used. In particular keys of 0, 64-255, and > 16361 are invalid.

This gives a total of 16,169 usable keys.
*/

constexpr uint16_t PID_PROFILE_INDEX_KEY = 0x0001;
constexpr uint16_t RATE_PROFILE_INDEX_KEY = 0x0002;
constexpr uint16_t ACC_CALIBRATION_STATE_KEY = 0x0003;
constexpr uint16_t GYRO_CALIBRATION_STATE_KEY = 0x0004;

static constexpr std::array<uint16_t, FlightController::PID_COUNT> PID_Keys = {
    // note these must go up in jumps of 4, since one key is used for each profile
    0x0100, 0x0104, 0x0108, 0x010C, 0x0110,
#if defined(USE_SIN_ANGLE_PIDS)
    0x0114, 0x011C
#endif
};
constexpr uint16_t MOTOR_MIXER_TYPE_KEY = 0x0005;

constexpr uint16_t ACC_OFFSET_KEY = 0x0200;
constexpr uint16_t GYRO_OFFSET_KEY = 0x0201;
constexpr uint16_t MAC_ADDRESS_KEY = 0x0202;

constexpr uint16_t DYNAMIC_NOTCH_FILTER_CONFIG_KEY = 0x0300;

// Part of PID profile
// Note that keys of items in PID profile must go up in jumps of 4, since 1 key is used for each profile
constexpr uint16_t FLIGHT_CONTROLLER_FILTERS_CONFIG_KEY = 0x0400;
constexpr uint16_t DYNAMIC_IDLE_CONTROLLER_CONFIG_KEY = 0x0404;
constexpr uint16_t FLIGHT_CONTROLLER_FLIGHTMODE_CONFIG_KEY = 0x408;
constexpr uint16_t FLIGHT_CONTROLLER_TPA_CONFIG_KEY = 0x40C;
constexpr uint16_t FLIGHT_CONTROLLER_AntiGravity_CONFIG_KEY = 0x0410;
constexpr uint16_t FLIGHT_CONTROLLER_DMAX_CONFIG_KEY = 0x0414;
constexpr uint16_t FLIGHT_CONTROLLER_ITERM_RELAX_CONFIG_KEY = 0x0418;
constexpr uint16_t FLIGHT_CONTROLLER_YAW_SPIN_RECOVERY_CONFIG_KEY = 0x041C;
constexpr uint16_t FLIGHT_CONTROLLER_CRASH_RECOVERY_CONFIG_KEY = 0x0420;
constexpr uint16_t FLIGHT_CONTROLLER_SIMPLIFIED_PID_settingsKey = 0x0424;
constexpr uint16_t OSD_CONFIG_KEY = 0x0428;
constexpr uint16_t OSD_ELEMENTS_CONFIG_KEY = 0x042C;

constexpr uint16_t RATES_KEY = 0x0500; // note jump of 4 to allow storage of 4 rates profiles

constexpr uint16_t IMU_FILTERS_CONFIG_KEY = 0x0600;
constexpr uint16_t RPM_FILTERS_CONFIG_KEY = 0x0601;
constexpr uint16_t FAILSAFE_CONFIG_KEY = 0x0602;
constexpr uint16_t RX_CONFIG_KEY = 0x0603;
constexpr uint16_t AUTOPILOT_CONFIG_KEY = 0x604;
constexpr uint16_t AUTOPILOT_POSITION_CONFIG_KEY = 0x605;
constexpr uint16_t ALTITUDE_HOLD_CONFIG_KEY = 0x606;
constexpr uint16_t MOTOR_CONFIG_KEY = 0x607;
constexpr uint16_t MOTOR_MIXER_CONFIG_KEY = 0x0608;
constexpr uint16_t VTX_CONFIG_KEY = 0x0609;
constexpr uint16_t GPS_CONFIG_KEY = 0x060A;
constexpr uint16_t FLIGHT_CONTROLLER_CRASH_FLIP_KEY = 0x060B;
constexpr uint16_t RC_MODES_ACTIVATION_CONDITIONS_KEY = 0x060C;
constexpr uint16_t RC_ADJUSTMENT_RANGES_KEY = 0x060D;
constexpr uint16_t FEATURES_CONFIG_KEY = 0x060E;


#if defined(USE_ARDUINO_ESP32_PREFERENCES)
static const char* nonVolatileStorageNamespace {"PTFL"}; // Protoflight
#endif


/*!
NOTE: NonVolatileStorage load functions return items by value. c++ uses Return Value Optimization (RVO)
so this is not inefficient.
*/
NonVolatileStorage::NonVolatileStorage(uint32_t flashMemorySize)
#if defined(USE_FLASH_KLV)
    : _flashKLV(flashMemorySize)
#endif
{
    (void)flashMemorySize;
}

NonVolatileStorage::NonVolatileStorage() :
    NonVolatileStorage(4096)
{
}

void NonVolatileStorage::init()
{
#if defined(USE_FLASH_KLV)
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        if (!_preferences.getBool("init")) {
            _preferences.putBool("init", true);
        }
        _preferences.end();
    }
#endif
}

int32_t NonVolatileStorage::clear()
{
#if defined(USE_FLASH_KLV)
    return OK;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.clear();
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    return OK;
#endif
}

void NonVolatileStorage::toHexChars(char* charPtr, uint16_t value)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
    *charPtr++ = '0';
    *charPtr++ = 'x';

    auto digit = static_cast<uint8_t>(value >> 12U);
    *charPtr++ = static_cast<char>((digit <= 9) ? digit +'0' : digit + 'A' - 10);
    digit = static_cast<uint8_t>((value >> 8U) & 0x000FU);
    *charPtr++ = static_cast<char>((digit <= 9) ? digit +'0' : digit + 'A' - 10);
    digit = static_cast<uint8_t>((value >> 4U) & 0x000FU);
    *charPtr++ = static_cast<char>((digit <= 9) ? digit +'0' : digit + 'A' - 10);
    digit = static_cast<uint8_t>(value & 0x000FU);
    *charPtr++ = static_cast<char>((digit <= 9) ? digit +'0' : digit + 'A' - 10);
    *charPtr = 0;
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
}

int32_t NonVolatileStorage::remove(uint16_t key)
{
#if defined(USE_FLASH_KLV)
    return _flashKLV.remove(key);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], key);
        _preferences.remove(&keyS[0]);
        _preferences.end();
    }
    return OK;
#else
    (void)key;
    return OK;
#endif
}


bool NonVolatileStorage::loadItem(uint16_t key, void* item, size_t length) const
{
#if defined(USE_FLASH_KLV)
    if (FlashKLV::OK == _flashKLV.read(item, length, key)) {
        return true;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], key);
        if (_preferences.isKey(&keyS[0])) {
            _preferences.getBytes(&keyS[0], item, length);
            _preferences.end();
            return true;
        }
        _preferences.end();
    }
#else
    (void)key;
    (void)item;
    (void)length;
#endif
    return false;
}

bool NonVolatileStorage::loadItem(uint16_t key, uint8_t pidProfileIndex, void* item, size_t length) const
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return false;
    }
    return loadItem(key + pidProfileIndex, item, length);
}

int32_t NonVolatileStorage::storeItem(uint16_t key, const void* item, size_t length, const void* defaults)
{
#if defined(USE_FLASH_KLV)
    if (!memcmp(defaults, item, length)) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, length, item);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], key);
        if (!memcmp(defaults, item, length)) {
            // value is the same as default, so no need to store it
            _preferences.remove(&keyS[0]);
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(&keyS[0], item, length);
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)key;
    (void)item;
    (void)length;
    (void)defaults;
    return ERROR_NOT_WRITTEN;
#endif
}

int32_t NonVolatileStorage::storeItem(uint16_t key, uint8_t pidProfileIndex, const void* item, size_t length, const void* defaults)
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    return storeItem(key + pidProfileIndex, item, length, defaults);
}


uint8_t NonVolatileStorage::loadPidProfileIndex() const
{
    uint8_t profileIndex {};
    if (loadItem(PID_PROFILE_INDEX_KEY, &profileIndex, sizeof(profileIndex))) { // cppcheck-suppress knownConditionTrueFalse
        return profileIndex;
    }
    return DEFAULT_PID_PROFILE;
}

int32_t NonVolatileStorage::storePidProfileIndex(uint8_t profileIndex)
{
    if (profileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint8_t defaultProfileIndex = DEFAULT_PID_PROFILE;
    return storeItem(PID_PROFILE_INDEX_KEY, &profileIndex, sizeof(profileIndex), &defaultProfileIndex);
}


uint8_t NonVolatileStorage::loadRateProfileIndex() const
{
    uint8_t profileIndex {};
    if (loadItem(RATE_PROFILE_INDEX_KEY, &profileIndex, sizeof(profileIndex))) { // cppcheck-suppress knownConditionTrueFalse
        return profileIndex;
    }
    return DEFAULT_RATE_PROFILE;
}

int32_t NonVolatileStorage::storeRateProfileIndex(uint8_t profileIndex)
{
    if (profileIndex >= RATE_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint8_t defaultProfileIndex = DEFAULT_RATE_PROFILE;
    return storeItem(RATE_PROFILE_INDEX_KEY, &profileIndex, sizeof(profileIndex), &defaultProfileIndex);
}


DynamicIdleController::config_t NonVolatileStorage::loadDynamicIdleControllerConfig(uint8_t pidProfileIndex) const
{
    {DynamicIdleController::config_t config {};
    if (loadItem(DYNAMIC_IDLE_CONTROLLER_CONFIG_KEY, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::dynamicIdleControllerConfig;
}

int32_t NonVolatileStorage::storeDynamicIdleControllerConfig(const DynamicIdleController::config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(DYNAMIC_IDLE_CONTROLLER_CONFIG_KEY, pidProfileIndex, &config, sizeof(config), &DEFAULTS::dynamicIdleControllerConfig);
}

MotorMixerBase::mixer_config_t NonVolatileStorage::loadMotorMixerConfig() const
{
    {MotorMixerBase::mixer_config_t config {};
    if (loadItem(DYNAMIC_IDLE_CONTROLLER_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::motorMixerConfig;
}

int32_t NonVolatileStorage::storeMotorMixerConfig(const MotorMixerBase::mixer_config_t& config)
{
    return storeItem(MOTOR_MIXER_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::motorMixerConfig);
}


MotorMixerBase::motor_config_t NonVolatileStorage::loadMotorConfig() const
{
    {MotorMixerBase::motor_config_t config {};
    if (loadItem(MOTOR_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::motorConfig;
}

int32_t NonVolatileStorage::storeMotorConfig(const MotorMixerBase::motor_config_t& config)
{
    return storeItem(MOTOR_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::motorConfig);
}


FlightController::filters_config_t NonVolatileStorage::loadFlightControllerFiltersConfig(uint8_t pidProfileIndex) const
{
    {FlightController::filters_config_t config {};
    if (loadItem(FLIGHT_CONTROLLER_FILTERS_CONFIG_KEY, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerFiltersConfig;
}

int32_t NonVolatileStorage::storeFlightControllerFiltersConfig(const FlightController::filters_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FLIGHT_CONTROLLER_FILTERS_CONFIG_KEY, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerFiltersConfig);
}


FlightController::flight_mode_config_t NonVolatileStorage::loadFlightControllerFlightModeConfig(uint8_t pidProfileIndex) const
{
    {FlightController::flight_mode_config_t config {};
    if (loadItem(FLIGHT_CONTROLLER_FLIGHTMODE_CONFIG_KEY, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerFlightModeConfig;
}

int32_t NonVolatileStorage::storeFlightControllerFlightModeConfig(const FlightController::flight_mode_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FLIGHT_CONTROLLER_FLIGHTMODE_CONFIG_KEY, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerFlightModeConfig);
}


FlightController::tpa_config_t NonVolatileStorage::loadFlightControllerTPA_Config(uint8_t pidProfileIndex) const
{
    {FlightController::tpa_config_t config {};
    if (loadItem(FLIGHT_CONTROLLER_TPA_CONFIG_KEY, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerTPA_Config;
}

int32_t NonVolatileStorage::storeFlightControllerTPA_Config(const FlightController::tpa_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FLIGHT_CONTROLLER_TPA_CONFIG_KEY, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerTPA_Config);
}


FlightController::anti_gravity_config_t NonVolatileStorage::loadFlightControllerAntiGravityConfig(uint8_t pidProfileIndex) const
{
    {FlightController::anti_gravity_config_t config {};
    if (loadItem(FLIGHT_CONTROLLER_AntiGravity_CONFIG_KEY, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerAntiGravityConfig;
}

int32_t NonVolatileStorage::storeFlightControllerAntiGravityConfig(const FlightController::anti_gravity_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FLIGHT_CONTROLLER_AntiGravity_CONFIG_KEY, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerAntiGravityConfig);
}


FlightController::crash_flip_config_t NonVolatileStorage::loadFlightControllerCrashFlipConfig() const
{
    {FlightController::crash_flip_config_t config {};
    if (loadItem(FLIGHT_CONTROLLER_CRASH_FLIP_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerCrashFlipConfig;
}

int32_t NonVolatileStorage::storeFlightControllerCrashFlipConfig(const FlightController::crash_flip_config_t& config)
{
    return storeItem(FLIGHT_CONTROLLER_CRASH_FLIP_KEY, &config, sizeof(config), &DEFAULTS::flightControllerCrashFlipConfig);
}


#if defined(USE_D_MAX)
FlightController::d_max_config_t NonVolatileStorage::loadFlightControllerDMaxConfig(uint8_t pidProfileIndex) const
{
    {FlightController::d_max_config_t config {};
    if (loadItem(FLIGHT_CONTROLLER_DMAX_CONFIG_KEY, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerDMaxConfig;
}

int32_t NonVolatileStorage::storeFlightControllerDMaxConfig(const FlightController::d_max_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FLIGHT_CONTROLLER_DMAX_CONFIG_KEY, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerDMaxConfig);
}
#endif


#if defined(USE_ITERM_RELAX)
FlightController::iterm_relax_config_t NonVolatileStorage::loadFlightControllerITermRelaxConfig(uint8_t pidProfileIndex) const
{
    {FlightController::iterm_relax_config_t config {};
    if (loadItem(FLIGHT_CONTROLLER_ITERM_RELAX_CONFIG_KEY, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerITermRelaxConfig;
}

int32_t NonVolatileStorage::storeFlightControllerITermRelaxConfig(const FlightController::iterm_relax_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FLIGHT_CONTROLLER_ITERM_RELAX_CONFIG_KEY, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerITermRelaxConfig);
}
#endif


#if defined(USE_YAW_SPIN_RECOVERY)
FlightController::yaw_spin_recovery_config_t NonVolatileStorage::loadFlightControllerYawSpinRecoveryConfig(uint8_t pidProfileIndex) const
{
    {FlightController::yaw_spin_recovery_config_t config {};
    if (loadItem(FLIGHT_CONTROLLER_YAW_SPIN_RECOVERY_CONFIG_KEY, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerYawSpinRecoveryConfig;
}

int32_t NonVolatileStorage::storeFlightControllerYawSpinRecoveryConfig(const FlightController::yaw_spin_recovery_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FLIGHT_CONTROLLER_YAW_SPIN_RECOVERY_CONFIG_KEY, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerYawSpinRecoveryConfig);
}
#endif


#if defined(USE_CRASH_RECOVERY)
FlightController::crash_recovery_config_t NonVolatileStorage::loadFlightControllerCrashRecoveryConfig(uint8_t pidProfileIndex) const
{
    {FlightController::crash_recovery_config_t config {};
    if (loadItem(FLIGHT_CONTROLLER_CRASH_RECOVERY_CONFIG_KEY, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerCrashRecoveryConfig;
}

int32_t NonVolatileStorage::storeFlightControllerCrashRecoveryConfig(const FlightController::crash_recovery_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FLIGHT_CONTROLLER_CRASH_RECOVERY_CONFIG_KEY, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerCrashRecoveryConfig);
}
#endif


#if defined(USE_DYNAMIC_NOTCH_FILTER)
DynamicNotchFilter::config_t NonVolatileStorage::loadDynamicNotchFilterConfig() const
{
    {DynamicNotchFilter::config_t config {};
    if (loadItem(DYNAMIC_NOTCH_FILTER_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::dynamicNotchFilterConfig;
}

int32_t NonVolatileStorage::storeDynamicNotchFilterConfig(const DynamicNotchFilter::config_t& config)
{
    return storeItem(DYNAMIC_NOTCH_FILTER_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::dynamicNotchFilterConfig);
}
#endif

#if defined(USE_RPM_FILTERS)
RpmFilters::config_t NonVolatileStorage::loadRPM_FiltersConfig() const
{
    {RpmFilters::config_t config {};
    if (loadItem(RPM_FILTERS_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::rpmFiltersConfig;
}

int32_t NonVolatileStorage::storeRPM_FiltersConfig(const RpmFilters::config_t& config)
{
    return storeItem(RPM_FILTERS_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::rpmFiltersConfig);
}
#endif
#if defined(USE_OSD)
OSD::config_t NonVolatileStorage::loadOSD_Config() const
{
    {OSD::config_t config {};
    if (loadItem(OSD_ELEMENTS_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::osdConfig;
}

int32_t NonVolatileStorage::storeOSD_Config(const OSD::config_t& config)
{
    return storeItem(OSD_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::osdConfig);
}

bool NonVolatileStorage::loadOSD_ElementsConfig(OSD_Elements::config_t& config) const
{
    if (loadItem(OSD_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return true;
    }
    return false;
}

int32_t NonVolatileStorage::storeOSD_ElementsConfig(const OSD_Elements::config_t& config)
{
    return storeItem(OSD_ELEMENTS_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::osdElementsConfig);
}
#endif
#if defined(USE_VTX)
VTX::config_t NonVolatileStorage::loadVTX_Config() const
{
    {VTX::config_t config {};
    if (loadItem(VTX_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::vtxConfig;
}
int32_t NonVolatileStorage::storeVTX_Config(const VTX::config_t& config)
{
    return storeItem(VTX_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::vtxConfig);
}
#endif
#if defined(USE_GPS)
GPS::config_t NonVolatileStorage::loadGPS_Config() const
{
    {GPS::config_t config {};
    if (loadItem(GPS_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::gpsConfig;
}
int32_t NonVolatileStorage::storeGPS_Config(const GPS::config_t& config)
{
    return storeItem(GPS_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::gpsConfig);
}
#endif
#if defined(USE_ALTITUDE_HOLD)
Autopilot::autopilot_config_t NonVolatileStorage::loadAutopilotConfig() const
{
    {Autopilot::autopilot_config_t config {};
    if (loadItem(AUTOPILOT_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::autopilotConfig;
}

int32_t NonVolatileStorage::storeAutopilotConfig(const Autopilot::autopilot_config_t& config)
{
    return storeItem(AUTOPILOT_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::autopilotConfig);
}

Autopilot::position_config_t NonVolatileStorage::loadAutopilotPositionConfig() const
{
    {Autopilot::position_config_t config {};
    if (loadItem(AUTOPILOT_POSITION_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::autopilotPositionConfig;
}

int32_t NonVolatileStorage::storeAutopilotPositionConfig(const Autopilot::position_config_t& config)
{
    return storeItem(AUTOPILOT_POSITION_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::autopilotPositionConfig);
}

Autopilot::altitude_hold_config_t NonVolatileStorage::loadAltitudeHoldConfig() const
{
    {Autopilot::altitude_hold_config_t config {};
    if (loadItem(ALTITUDE_HOLD_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::autopilotAltitudeHoldConfig;
}

int32_t NonVolatileStorage::storeAltitudeHoldConfig(const Autopilot::altitude_hold_config_t& config)
{
    return storeItem(ALTITUDE_HOLD_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::autopilotAltitudeHoldConfig);
}
#endif


IMU_Filters::config_t NonVolatileStorage::loadIMU_FiltersConfig() const
{
    {IMU_Filters::config_t config {};
    if (loadItem(IMU_FILTERS_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::imuFiltersConfig;
}

int32_t NonVolatileStorage::storeIMU_FiltersConfig(const IMU_Filters::config_t& config)
{
    return storeItem(IMU_FILTERS_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::imuFiltersConfig);
}


Cockpit::failsafe_config_t NonVolatileStorage::loadFailsafeConfig() const
{
    {Cockpit::failsafe_config_t config {};
    if (loadItem(FAILSAFE_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::cockpitFailSafeConfig;
}

int32_t NonVolatileStorage::storeFailsafeConfig(const Cockpit::failsafe_config_t& config)
{
    return storeItem(FAILSAFE_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::cockpitFailSafeConfig);
}

Features::config_t NonVolatileStorage::loadFeaturesConfig() const
{
    {Features::config_t config {};
    if (loadItem(FEATURES_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::featuresConfig;
}

int32_t NonVolatileStorage::storeFeaturesConfig(const Features::config_t& config)
{
    return storeItem(FEATURES_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::featuresConfig);
}

RX::config_t NonVolatileStorage::loadRX_Config() const
{
    {RX::config_t config {};
    if (loadItem(RX_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::RX_Config;
}

int32_t NonVolatileStorage::storeRX_Config(const RX::config_t& config)
{
    return storeItem(RX_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::RX_Config);
}


rc_modes_activation_condition_array_t NonVolatileStorage::load_rc_mode_activation_conditions() const
{
    {rc_modes_activation_condition_array_t mode_activation_conditions {};
    if (loadItem(RC_MODES_ACTIVATION_CONDITIONS_KEY, &mode_activation_conditions, sizeof(mode_activation_conditions))) {
        return mode_activation_conditions;
    }}
    return DEFAULTS::RC_MODE_ACTIVATION_CONDITIONS;
}

int32_t NonVolatileStorage::store_rc_mode_activation_conditions(const rc_modes_activation_condition_array_t& mode_activation_conditions)
{
    return storeItem(RC_MODES_ACTIVATION_CONDITIONS_KEY, &mode_activation_conditions, sizeof(mode_activation_conditions), &DEFAULTS::RC_MODE_ACTIVATION_CONDITIONS);
}

#if defined(USE_RC_ADJUSTMENTS)
RC_Adjustments::adjustment_ranges_t NonVolatileStorage::loadRC_AdjustmentRanges() const
{
    {RC_Adjustments::adjustment_ranges_t adjustmentRanges {};
    if (loadItem(RC_ADJUSTMENT_RANGES_KEY, &adjustmentRanges, sizeof(adjustmentRanges))) {
        return adjustmentRanges;
    }}
    return DEFAULTS::RC_AdjustmentRanges;
}

int32_t NonVolatileStorage::storeRC_AdjustmentRanges(const RC_Adjustments::adjustment_ranges_t& adjustmentRanges)
{
    return storeItem(RC_ADJUSTMENT_RANGES_KEY, &adjustmentRanges, sizeof(adjustmentRanges), &DEFAULTS::RC_AdjustmentRanges);
}
#endif

rates_t NonVolatileStorage::loadRates(uint8_t rateProfileIndex) const
{
    {rates_t rates {};
    if (rateProfileIndex < RATE_PROFILE_COUNT && loadItem(RATES_KEY + rateProfileIndex, &rates, sizeof(rates))) { // cppcheck-suppress knownConditionTrueFalse
        return rates;
    }}
    return DEFAULTS::cockpitRates;
}

int32_t NonVolatileStorage::storeRates(const rates_t& rates, uint8_t rateProfileIndex)
{
    if (rateProfileIndex >= RATE_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint16_t key = RATES_KEY + rateProfileIndex;
    return storeItem(key, &rates, sizeof(rates), &DEFAULTS::cockpitRates);
}

VehicleControllerBase::PIDF_uint16_t NonVolatileStorage::loadPID(uint8_t pidIndex, uint8_t pidProfileIndex) const
{
    assert(pidIndex <= FlightController::PID_COUNT);
    {VehicleControllerBase::PIDF_uint16_t pid {};
    if (pidProfileIndex < PID_PROFILE_COUNT && loadItem(PID_Keys[pidIndex] + pidProfileIndex, &pid, sizeof(pid))) { // cppcheck-suppress knownConditionTrueFalse
        return pid;
    }}
    return DEFAULTS::flightControllerPIDs[pidIndex];
}

int32_t NonVolatileStorage::storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex, uint8_t pidProfileIndex)
{
    assert(pidIndex <= FlightController::PID_COUNT);
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint16_t key = PID_Keys[pidIndex] + pidProfileIndex;
    return storeItem(key, &pid, sizeof(pid), &DEFAULTS::flightControllerPIDs[pidIndex]);
}

void NonVolatileStorage::resetPID(uint8_t pidIndex, uint8_t pidProfileIndex)
{
    assert(pidIndex <= FlightController::PID_COUNT);
    assert(pidProfileIndex < PID_PROFILE_COUNT);
    remove(PID_Keys[pidIndex]);
    (void)pidProfileIndex; //!!TODO: check if this is needed
}

FlightController::simplified_pid_settings_t NonVolatileStorage::loadSimplifiedPID_settings(uint8_t pidProfileIndex) const
{
    assert(pidProfileIndex < PID_PROFILE_COUNT);
    {FlightController::simplified_pid_settings_t settings {};
    if (pidProfileIndex < PID_PROFILE_COUNT && loadItem(FLIGHT_CONTROLLER_SIMPLIFIED_PID_settingsKey + pidProfileIndex, &settings, sizeof(settings))) { // cppcheck-suppress knownConditionTrueFalse
        return settings;
    }}
    return DEFAULTS::flightControllerSimplifiedPID_settings;
}

int32_t NonVolatileStorage::storeSimplifiedPID_settings(const FlightController::simplified_pid_settings_t& settings, uint8_t pidProfileIndex)
{
    assert(pidProfileIndex < PID_PROFILE_COUNT);
    const uint16_t key = FLIGHT_CONTROLLER_SIMPLIFIED_PID_settingsKey + pidProfileIndex;
    return storeItem(key, &settings, sizeof(settings), &DEFAULTS::flightControllerSimplifiedPID_settings);
}


NonVolatileStorage::calibration_state_e NonVolatileStorage::loadAccCalibrationState() const
{
    calibration_state_e calibrationState {};
    if (loadItem(ACC_CALIBRATION_STATE_KEY, &calibrationState, sizeof(calibrationState))) { // cppcheck-suppress knownConditionTrueFalse
        return calibrationState;
    }
    return NOT_CALIBRATED;
}

int32_t NonVolatileStorage::storeAccCalibrationState(calibration_state_e calibrationState)
{
    const calibration_state_e defaultCalibrationState = NOT_CALIBRATED;
    return storeItem(ACC_CALIBRATION_STATE_KEY, &calibrationState, sizeof(calibrationState), &defaultCalibrationState);
}

xyz_t NonVolatileStorage::loadAccOffset() const
{
    {xyz_t offset {};
    if (loadItem(ACC_OFFSET_KEY, &offset, sizeof(offset))) { // cppcheck-suppress knownConditionTrueFalse
        return offset;
    }}
    return xyz_t { .x = 0.0F, .y = 0.0F, .z = 0.0F };
}

int32_t NonVolatileStorage::storeAccOffset(const xyz_t& offset)
{
    const xyz_t defaultOffset = { .x = 0.0F, .y = 0.0F, .z = 0.0F };
    return storeItem(ACC_OFFSET_KEY, &offset, sizeof(offset), &defaultOffset);
}

NonVolatileStorage::calibration_state_e NonVolatileStorage::loadGyroCalibrationState() const
{
    calibration_state_e calibrationState {};
    if (loadItem(GYRO_CALIBRATION_STATE_KEY, &calibrationState, sizeof(calibrationState))) { // cppcheck-suppress knownConditionTrueFalse
        return calibrationState;
    }
    return NOT_CALIBRATED;
}

int32_t NonVolatileStorage::storeGyroCalibrationState(calibration_state_e calibrationState)
{
    const calibration_state_e defaultCalibrationState = NOT_CALIBRATED;
    return storeItem(GYRO_CALIBRATION_STATE_KEY, &calibrationState, sizeof(calibrationState), &defaultCalibrationState);
}

xyz_t NonVolatileStorage::loadGyroOffset() const
{
    {xyz_t offset {};
    if (loadItem(GYRO_OFFSET_KEY, &offset, sizeof(offset))) { // cppcheck-suppress knownConditionTrueFalse
        return offset;
    }}
    return xyz_t { .x = 0.0F, .y = 0.0F, .z = 0.0F };
}

int32_t NonVolatileStorage::storeGyroOffset(const xyz_t& offset)
{
    const xyz_t defaultOffset = { .x = 0.0F, .y = 0.0F, .z = 0.0F };
    return storeItem(GYRO_OFFSET_KEY, &offset, sizeof(offset), &defaultOffset);
}

void NonVolatileStorage::loadMacAddress(uint8_t* macAddress) const // NOLINT(readability-non-const-parameter)
{
#if defined(USE_FLASH_KLV)
    (void)macAddress;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], MAC_ADDRESS_KEY);
        _preferences.getBytes(&keyS[0], macAddress, MAC_ADDRESS_LEN);
        _preferences.end();
    }
#else
    (void)macAddress;
#endif
}

int32_t NonVolatileStorage::storeMacAddress(const uint8_t* macAddress)
{
#if defined(USE_FLASH_KLV)
    (void)macAddress;
    return OK;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], MAC_ADDRESS_KEY);
        _preferences.putBytes(&keyS[0], macAddress, MAC_ADDRESS_LEN);
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)macAddress;
    return OK;
#endif
}

int32_t NonVolatileStorage::storeAll(const IMU_Filters& imuFilters, const FlightController& flightController, const Cockpit& cockpit, uint8_t pidProfile, uint8_t ratesProfile)
{
#if defined(USE_DYNAMIC_IDLE)
    const DynamicIdleController* dynamicIdleController = flightController.getMotorMixer().get_dynamic_idle_controller();
    if (dynamicIdleController) {
        storeDynamicIdleControllerConfig(dynamicIdleController->get_config(), pidProfile);
    }
#endif
#if defined(USE_RPM_FILTERS)
    const RpmFilters* rpmFilters = flightController.getMotorMixer().get_rpm_filters();
    if (rpmFilters) {
        storeRPM_FiltersConfig(rpmFilters->get_config());
    }
#endif

    storeFlightControllerFiltersConfig(flightController.getFiltersConfig(), pidProfile);

    storeFlightControllerFlightModeConfig(flightController.getFlightModeConfig(), pidProfile);

    storeFlightControllerTPA_Config(flightController.getTPA_Config(), pidProfile);

    storeFlightControllerAntiGravityConfig(flightController.getAntiGravityConfig(), pidProfile);

    storeFlightControllerCrashFlipConfig(flightController.getCrashFlipConfig());

#if defined(USE_D_MAX)
    storeFlightControllerDMaxConfig(flightController.getDMaxConfig(), pidProfile);
#endif
#if defined(USE_ITERM_RELAX)
    storeFlightControllerITermRelaxConfig(flightController.getITermRelaxConfig(), pidProfile);
#endif
#if defined(USE_YAW_SPIN_RECOVERY)
    storeFlightControllerYawSpinRecoveryConfig(flightController.getYawSpinRecoveryConfig(), pidProfile);
#endif
#if defined(USE_CRASH_RECOVERY)
    storeFlightControllerCrashRecoveryConfig(flightController.getCrashRecoveryConfig(), pidProfile);
#endif
#if defined(USE_ALTITUDE_HOLD)
    storeAltitudeHoldConfig(cockpit.getAutopilot().getAltitudeHoldConfig());
#endif

    const IMU_Filters::config_t imuFiltersConfig = imuFilters.getConfig();
    storeIMU_FiltersConfig(imuFiltersConfig);

#if defined(USE_DYNAMIC_NOTCH_FILTER)
    storeDynamicNotchFilterConfig(imuFilters.getDynamicNotchFilterConfig());
#endif

    storeRates(cockpit.getRates(), ratesProfile);
    store_rc_mode_activation_conditions(cockpit.get_rc_modes().get_mode_activation_conditions());
#if defined(USE_RC_ADJUSTMENTS)
    storeRC_AdjustmentRanges(cockpit.getRC_Adjustments().getAdjustmentRanges());
#endif
    storeFeaturesConfig(cockpit.getFeaturesConfig());

    return OK;
}

void Cockpit::setCurrentRateProfileIndex(uint8_t currentRateProfileIndex)
{
    _currentRateProfileIndex = currentRateProfileIndex;
    _rates = _nvs.loadRates(currentRateProfileIndex);
}

void Cockpit::setCurrentPidProfileIndex(uint8_t currentPidProfileIndex)
{
    _currentPidProfileIndex = currentPidProfileIndex;
}

#if defined(USE_CMS)
void CMSX::saveConfigAndNotify()
{
    Cockpit& cockpit = _cms.getCockpitMutable();
    NonVolatileStorage& nvs = cockpit.getNonVolatileStorage();

    nvs.storeAll(_imuFilters, cockpit.getFlightController(), cockpit, cockpit.getCurrentPidProfileIndex(), cockpit.getCurrentRateProfileIndex());
}
#endif
