#include"NonVolatileStorage.h"
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

constexpr uint16_t PID_ProfileIndexKey = 0x0001;
constexpr uint16_t RateProfileIndexKey = 0x0002;
static const std::array<uint16_t, FlightController::PID_COUNT> PID_Keys = {
    // note these must go up in jumps of 4, since one key is used for each profile
    0x0100, 0x0104, 0x0108, 0x010C, 0x0110, 0x0114, 0x011C
};
constexpr uint16_t MotorMixerTypeKey = 0x0003;

constexpr uint16_t AccOffsetKey = 0x0200;
constexpr uint16_t GyroOffsetKey = 0x0201;
constexpr uint16_t MacAddressKey = 0x0202;

constexpr uint16_t DynamicNotchFilterConfigKey = 0x0300;

// Part of PID profile
// Note that keys of items in PID profile must go up in jumps of 4, since 1 key is used for each profile
constexpr uint16_t FlightControllerFiltersConfigKey = 0x0400;
constexpr uint16_t DynamicIdleControllerConfigKey = 0x0404;
constexpr uint16_t FlightControllerFlightModeConfigKey = 0x408;
constexpr uint16_t FlightControllerTPA_ConfigKey = 0x40C;
constexpr uint16_t FlightControllerAntiGravityConfigKey = 0x0410;
constexpr uint16_t FlightControllerDMaxConfigKey = 0x0414;
constexpr uint16_t FlightControllerITermRelaxConfigKey = 0x0418;
constexpr uint16_t FlightControllerYawSpinRecoveryConfigKey = 0x041C;
constexpr uint16_t FlightControllerCrashRecoveryConfigKey = 0x0420;

constexpr uint16_t RadioControllerRatesKey = 0x0500; // note jump of 4 to allow storage of 4 rates profiles
constexpr uint16_t IMU_FiltersConfigKey = 0x0504;
constexpr uint16_t RPM_FiltersConfigKey = 0x0505;
constexpr uint16_t RadioControllerFailsafeKey = 0x0506;

#if defined(USE_ARDUINO_ESP32_PREFERENCES)
static const char* nonVolatileStorageNamespace {"PTFL"}; // ProtoFlight
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


MotorMixerBase::type_e NonVolatileStorage::loadMotorMixerType() const
{
    MotorMixerBase::type_e motorMixerType {};
    if (loadItem(MotorMixerTypeKey, &motorMixerType, sizeof(motorMixerType))) { // cppcheck-suppress knownConditionTrueFalse
        return motorMixerType;
    }
    return DEFAULTS::motorMixerType;
}

int32_t NonVolatileStorage::storeMotorMixerType(MotorMixerBase::type_e motorMixerType)
{
    MotorMixerBase::type_e defaultMotorMixerType = DEFAULTS::motorMixerType;
    return storeItem(PID_ProfileIndexKey, &motorMixerType, sizeof(motorMixerType), &defaultMotorMixerType);
}


uint8_t NonVolatileStorage::loadPidProfileIndex() const
{
    uint8_t profileIndex {};
    if (loadItem(PID_ProfileIndexKey, &profileIndex, sizeof(profileIndex))) { // cppcheck-suppress knownConditionTrueFalse
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
    return storeItem(PID_ProfileIndexKey, &profileIndex, sizeof(profileIndex), &defaultProfileIndex);
}


uint8_t NonVolatileStorage::loadRateProfileIndex() const
{
    uint8_t profileIndex {};
    if (loadItem(RateProfileIndexKey, &profileIndex, sizeof(profileIndex))) { // cppcheck-suppress knownConditionTrueFalse
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
    return storeItem(RateProfileIndexKey, &profileIndex, sizeof(profileIndex), &defaultProfileIndex);
}


DynamicIdleController::config_t NonVolatileStorage::loadDynamicIdleControllerConfig(uint8_t pidProfileIndex) const
{
    {DynamicIdleController::config_t config {};
    if (loadItem(DynamicIdleControllerConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::dynamicIdleControllerConfig;
}

int32_t NonVolatileStorage::storeDynamicIdleControllerConfig(const DynamicIdleController::config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(DynamicIdleControllerConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::dynamicIdleControllerConfig);
}


FlightController::filters_config_t NonVolatileStorage::loadFlightControllerFiltersConfig(uint8_t pidProfileIndex) const
{
    {FlightController::filters_config_t config {};
    if (loadItem(FlightControllerFiltersConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerFiltersConfig;
}

int32_t NonVolatileStorage::storeFlightControllerFiltersConfig(const FlightController::filters_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerFiltersConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerFiltersConfig);
}


FlightController::flight_mode_config_t NonVolatileStorage::loadFlightControllerFlightModeConfig(uint8_t pidProfileIndex) const
{
    {FlightController::flight_mode_config_t config {};
    if (loadItem(FlightControllerFlightModeConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerFlightModeConfig;
}

int32_t NonVolatileStorage::storeFlightControllerFlightModeConfig(const FlightController::flight_mode_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerFlightModeConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerFlightModeConfig);
}


FlightController::tpa_config_t NonVolatileStorage::loadFlightControllerTPA_Config(uint8_t pidProfileIndex) const
{
    {FlightController::tpa_config_t config {};
    if (loadItem(FlightControllerTPA_ConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerTPA_Config;
}

int32_t NonVolatileStorage::storeFlightControllerTPA_Config(const FlightController::tpa_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerTPA_ConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerTPA_Config);
}


FlightController::anti_gravity_config_t NonVolatileStorage::loadFlightControllerAntiGravityConfig(uint8_t pidProfileIndex) const
{
    {FlightController::anti_gravity_config_t config {};
    if (loadItem(FlightControllerAntiGravityConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerAntiGravityConfig;
}

int32_t NonVolatileStorage::storeFlightControllerAntiGravityConfig(const FlightController::anti_gravity_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerAntiGravityConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerAntiGravityConfig);
}


#if defined(USE_D_MAX)
FlightController::d_max_config_t NonVolatileStorage::loadFlightControllerDMaxConfig(uint8_t pidProfileIndex) const
{
    {FlightController::d_max_config_t config {};
    if (loadItem(FlightControllerDMaxConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerDMaxConfig;
}

int32_t NonVolatileStorage::storeFlightControllerDMaxConfig(const FlightController::d_max_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerDMaxConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerDMaxConfig);
}
#endif


#if defined(USE_ITERM_RELAX)
FlightController::iterm_relax_config_t NonVolatileStorage::loadFlightControllerITermRelaxConfig(uint8_t pidProfileIndex) const
{
    {FlightController::iterm_relax_config_t config {};
    if (loadItem(FlightControllerITermRelaxConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerITermRelaxConfig;
}

int32_t NonVolatileStorage::storeFlightControllerITermRelaxConfig(const FlightController::iterm_relax_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerITermRelaxConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerITermRelaxConfig);
}
#endif


#if defined(USE_YAW_SPIN_RECOVERY)
FlightController::yaw_spin_recovery_config_t NonVolatileStorage::loadFlightControllerYawSpinRecoveryConfig(uint8_t pidProfileIndex) const
{
    {FlightController::yaw_spin_recovery_config_t config {};
    if (loadItem(FlightControllerYawSpinRecoveryConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerYawSpinRecoveryConfig;
}

int32_t NonVolatileStorage::storeFlightControllerYawSpinRecoveryConfig(const FlightController::yaw_spin_recovery_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerYawSpinRecoveryConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerYawSpinRecoveryConfig);
}
#endif


#if defined(USE_CRASH_RECOVERY)
FlightController::crash_recovery_config_t NonVolatileStorage::loadFlightControllerCrashRecoveryConfig(uint8_t pidProfileIndex) const
{
    {FlightController::crash_recovery_config_t config {};
    if (loadItem(FlightControllerCrashRecoveryConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::flightControllerCrashRecoveryConfig;
}

int32_t NonVolatileStorage::storeFlightControllerCrashRecoveryConfig(const FlightController::crash_recovery_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerCrashRecoveryConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerCrashRecoveryConfig);
}
#endif


#if defined(USE_DYNAMIC_NOTCH_FILTER)
DynamicNotchFilter::config_t NonVolatileStorage::loadDynamicNotchFilterConfig() const
{
    {DynamicNotchFilter::config_t config {};
    if (loadItem(DynamicNotchFilterConfigKey, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::dynamicNotchFilterConfig;
}

int32_t NonVolatileStorage::storeDynamicNotchFilterConfig(const DynamicNotchFilter::config_t& config)
{
    return storeItem(DynamicNotchFilterConfigKey, &config, sizeof(config), &DEFAULTS::dynamicNotchFilterConfig);
}
#endif

IMU_Filters::config_t NonVolatileStorage::loadIMU_FiltersConfig() const
{
    {IMU_Filters::config_t config {};
    if (loadItem(IMU_FiltersConfigKey, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::imuFiltersConfig;
}

int32_t NonVolatileStorage::storeIMU_FiltersConfig(const IMU_Filters::config_t& config)
{
    return storeItem(IMU_FiltersConfigKey, &config, sizeof(config), &DEFAULTS::imuFiltersConfig);
}


#if defined(USE_RPM_FILTERS)
RPM_Filters::config_t NonVolatileStorage::loadRPM_FiltersConfig() const
{
    {RPM_Filters::config_t config {};
    if (loadItem(RPM_FiltersConfigKey, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::rpmFiltersConfig;
}

int32_t NonVolatileStorage::storeRPM_FiltersConfig(const RPM_Filters::config_t& config)
{
    return storeItem(RPM_FiltersConfigKey, &config, sizeof(config), &DEFAULTS::rpmFiltersConfig);
}
#endif


RadioController::failsafe_t NonVolatileStorage::loadRadioControllerFailsafe() // NOLINT(readability-make-member-function-const)
{
    {RadioController::failsafe_t failsafe {};
    if (loadItem(RadioControllerFailsafeKey, &failsafe, sizeof(failsafe))) { // cppcheck-suppress knownConditionTrueFalse
    }}
    return DEFAULTS::radioControllerFailsafe;
}

int32_t NonVolatileStorage::storeRadioControllerFailsafe(const RadioController::failsafe_t& failsafe)
{
    return storeItem(RadioControllerFailsafeKey, &failsafe, sizeof(failsafe), &DEFAULTS::radioControllerFailsafe);
}

RadioController::rates_t NonVolatileStorage::loadRadioControllerRates(uint8_t rateProfileIndex) const
{
    {RadioController::rates_t rates {};
    if (rateProfileIndex < RATE_PROFILE_COUNT && loadItem(RadioControllerRatesKey + rateProfileIndex, &rates, sizeof(rates))) { // cppcheck-suppress knownConditionTrueFalse
        return rates;
    }}
    return DEFAULTS::radioControllerRates;
}

int32_t NonVolatileStorage::storeRadioControllerRates(const RadioController::rates_t& rates, uint8_t rateProfileIndex)
{
    if (rateProfileIndex >= RATE_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint16_t key = RadioControllerRatesKey + rateProfileIndex;
    return storeItem(key, &rates, sizeof(rates), &DEFAULTS::radioControllerRates);
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
    return storeItem(key, &key, sizeof(pid), &DEFAULTS::flightControllerPIDs[pidIndex]);
}

void NonVolatileStorage::resetPID(uint8_t pidIndex, uint8_t pidProfileIndex)
{
    assert(pidIndex <= FlightController::PID_COUNT);
    assert(pidProfileIndex < PID_PROFILE_COUNT);
    remove(PID_Keys[pidIndex]);
    (void)pidProfileIndex; //!!TODO: check if this is needed
}


bool NonVolatileStorage::loadAccOffset(int32_t& x, int32_t& y, int32_t& z) const
{
    xyz_int32_t xyz {};
    if (loadItem(AccOffsetKey, &xyz, sizeof(xyz))) { // cppcheck-suppress knownConditionTrueFalse
        x = xyz.x;
        y = xyz.y;
        z = xyz.z;
        return true;
    }
    return false;
}

int32_t NonVolatileStorage::storeAccOffset(int32_t x, int32_t y, int32_t z)
{
    const xyz_int32_t xyz = { .x = x, .y = y, .z = z };
    const xyz_int32_t xyzDefault = { .x = 0, .y = 0, .z = 0 };
    return storeItem(AccOffsetKey, &xyz, sizeof(xyz), &xyzDefault);
}

bool NonVolatileStorage::loadGyroOffset(int32_t& x, int32_t& y, int32_t& z) const
{
    xyz_int32_t xyz {};
    if (loadItem(GyroOffsetKey, &xyz, sizeof(xyz))) { // cppcheck-suppress knownConditionTrueFalse
        x = xyz.x;
        y = xyz.y;
        z = xyz.z;
        return true;
    }
    return false;
}

int32_t NonVolatileStorage::storeGyroOffset(int32_t x, int32_t y, int32_t z)
{
    const xyz_int32_t xyz = { .x = x, .y = y, .z = z };
    const xyz_int32_t xyzDefault = { .x = 0, .y = 0, .z = 0 };
    return storeItem(GyroOffsetKey, &xyz, sizeof(xyz), &xyzDefault);
}

void NonVolatileStorage::loadMacAddress(uint8_t* macAddress) const // NOLINT(readability-non-const-parameter)
{
#if defined(USE_FLASH_KLV)
    (void)macAddress;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], MacAddressKey);
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
        toHexChars(&keyS[0], MacAddressKey);
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

int32_t NonVolatileStorage::storeAll(const FlightController& flightController, const RadioController& radioController, uint8_t pidProfile, uint8_t ratesProfile)
{
#if defined(USE_DYNAMIC_IDLE)
    const DynamicIdleController* dynamicIdleController = flightController.getMixer().getDynamicIdleController();
    if (dynamicIdleController) {
        const DynamicIdleController::config_t dynamicIdleControllerConfig = dynamicIdleController->getConfig();
        storeDynamicIdleControllerConfig(dynamicIdleControllerConfig, pidProfile);
    }
#endif

    storeFlightControllerFiltersConfig(flightController.getFiltersConfig(), pidProfile);

    storeFlightControllerFlightModeConfig(flightController.getFlightModeConfig(), pidProfile);

    storeFlightControllerTPA_Config(flightController.getTPA_Config(), pidProfile);

    storeFlightControllerAntiGravityConfig(flightController.getAntiGravityConfig(), pidProfile);

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

    const AHRS& ahrs = flightController.getAHRS();
    const IMU_Filters& imuFilters = static_cast<IMU_Filters&>(ahrs.getIMU_Filters()); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    const IMU_Filters::config_t imuFiltersConfig = imuFilters.getConfig();
    storeIMU_FiltersConfig(imuFiltersConfig);

#if defined(USE_RPM_FILTERS)
    storeRPM_FiltersConfig(imuFilters.getRPM_FiltersConfig());
#endif
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    storeDynamicNotchFilterConfig(imuFilters.getDynamicNotchFilterConfig());
#endif

    storeRadioControllerRates(radioController.getRates(), ratesProfile);

    return OK;
}
