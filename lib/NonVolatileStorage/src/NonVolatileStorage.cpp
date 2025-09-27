#include"NonVolatileStorage.h"
#include <AHRS.h>
#include <cstring>


#if defined(USE_FLASH_KLV)
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
// for FlashKLV, keys must be in range [0x0100, 0x3FFF]
static constexpr uint16_t PID_ProfileIndexKey = 0x0001;
static constexpr uint16_t RateProfileIndexKey = 0x0002;
static const std::array<uint16_t, FlightController::PID_COUNT> PID_Keys = {
    // note these must go up in jumps of 4, since one key is used for each profile
    0x0100, 0x0104, 0x0108, 0x010C, 0x0110, 0x0114, 0x011C
};
static constexpr uint16_t AccOffsetKey = 0x0200;
static constexpr uint16_t GyroOffsetKey = 0x0201;
static constexpr uint16_t MacAddressKey = 0x0202;
// Part of PID profile
// Note that keys of items in PID profile must go up in jumps of 4, since 1 key is used for each profile
static constexpr uint16_t DynamicIdleControllerConfigKey = 0x0400;
static constexpr uint16_t FlightControllerFiltersConfigKey = 0x0404;
static constexpr uint16_t FlightControllerAntiGravityConfigKey = 0x0408;
static constexpr uint16_t FlightControllerDMaxConfigKey = 0x040C;

static constexpr uint16_t RadioControllerRatesKey = 0x0500;
static constexpr uint16_t IMU_FiltersConfigKey = 0x0504;
static constexpr uint16_t RPM_FiltersConfigKey = 0x0505;
static constexpr uint16_t RadioControllerFailsafeKey = 0x0506;

#elif defined(USE_ARDUINO_ESP32_PREFERENCES)

static const char* nonVolatileStorageNamespace {"PTFL"}; // ProtoFlight
static const std::array<std::string, FlightController::PID_COUNT> PID_Keys = {
    "RoRa",
    "PiRa",
    "YaRa",
    "RoAn",
    "PiAn"
    "RoSA",
    "PiSA"
};

static const char* PID_ProfileIndexKey = "PPI";
static const char* RateProfileIndexKey = "RPI";
static const char* DynamicIdleControllerConfigKey = "DIC";
static const char* FlightControllerFiltersConfigKey = "FCF";
static const char* FlightControllerAntiGravityConfigKey = "FCAG";
static const char* FlightControllerDMaxConfigKey = "FCDM";
static const char* IMU_FiltersConfigKey = "IF";
static const char* RPM_FiltersConfigKey = "RPM";
static const char* RadioControllerFailsafeKey = "RCFS";
static const char* RadioControllerRatesKey = "RCR";
static const char* AccOffsetKey = "ACC";
static const char* GyroOffsetKey = "GYR";
static const char* MacAddressKey = "MAC";
#endif

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

#if defined(USE_FLASH_KLV)
int32_t NonVolatileStorage::remove(uint16_t key)
{
    return _flashKLV.remove(key);
}

#else

int32_t NonVolatileStorage::remove(const std::string& name)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.remove(name.c_str());
    }
#else
    (void)name;
#endif
    return OK;
}
#endif

uint8_t NonVolatileStorage::loadPidProfileIndex() const
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = PID_ProfileIndexKey;
    uint8_t profileIndex {}; // NOLINT(misc-const-correctness) false positive
    if (FlashKLV::OK == _flashKLV.read(&profileIndex, sizeof(profileIndex), key)) {
        return profileIndex;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = PID_ProfileIndexKey;
        if (_preferences.isKey(key.c_str())) {
            uint8_t profileIndex {}; // NOLINT(misc-const-correctness) false positive
            _preferences.getBytes(key.c_str(), &profileIndex, sizeof(profileIndex));
            _preferences.end();
            return profileIndex;
        }
        _preferences.end();
    }
#endif
    return DEFAULT_PID_PROFILE;
}

int32_t NonVolatileStorage::storePidProfileIndex(uint8_t profileIndex)
{
    if (profileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = PID_ProfileIndexKey;
    if (profileIndex == DEFAULT_PID_PROFILE) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(profileIndex), &profileIndex);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = PID_ProfileIndexKey;
        if (profileIndex == DEFAULT_PID_PROFILE) {
            // value is the same as default, so no need to store it
            _preferences.remove(key.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(key.c_str(), &profileIndex, sizeof(profileIndex));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)profileIndex;
    return OK;
#endif
}

uint8_t NonVolatileStorage::loadRateProfileIndex() const
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = RateProfileIndexKey;
    uint8_t profileIndex {}; // NOLINT(misc-const-correctness) false positive
    if (FlashKLV::OK == _flashKLV.read(&profileIndex, sizeof(profileIndex), key)) {
        return profileIndex;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = RateProfileIndexKey;
        if (_preferences.isKey(key.c_str())) {
            uint8_t profileIndex {}; // NOLINT(misc-const-correctness) false positive
            _preferences.getBytes(key.c_str(), &profileIndex, sizeof(profileIndex));
            _preferences.end();
            return profileIndex;
        }
        _preferences.end();
    }
#endif
    return DEFAULT_RATE_PROFILE;
}

int32_t NonVolatileStorage::storeRateProfileIndex(uint8_t profileIndex)
{
    if (profileIndex >= RATE_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = RateProfileIndexKey;
    if (profileIndex == DEFAULT_PID_PROFILE) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(profileIndex), &profileIndex);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = RateProfileIndexKey;
        if (profileIndex == DEFAULT_RATE_PROFILE) {
            // value is the same as default, so no need to store it
            _preferences.remove(key.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(key.c_str(), &profileIndex, sizeof(profileIndex));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)profileIndex;
    return OK;
#endif
}

#if !defined(USE_FLASH_KLV)
bool NonVolatileStorage::loadItem(const char* key, uint8_t pidProfileIndex, void* item, size_t length) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string itemKey = key + ('0' + pidProfileIndex);
        if (_preferences.isKey(itemKey.c_str())) {
            _preferences.getBytes(itemKey.c_str(), item, length);
            _preferences.end();
            return true;
        }
        _preferences.end();
    }
#else
    (void)key;
    (void)pidProfileIndex;
    (void)item;
    (void)length;
#endif
    return false;
}

bool NonVolatileStorage::loadItem(const char* key, void* item, size_t length) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string itemKey = key;
        if (_preferences.isKey(itemKey.c_str())) {
            _preferences.getBytes(itemKey.c_str(), item, length);
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

int32_t NonVolatileStorage::storeItem(const char* key, const void* item, size_t length, const void* defaults)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string itemKey = key;
        if (!memcmp(defaults, item, length)) {
            // value is the same as default, so no need to store it
            _preferences.remove(itemKey.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(itemKey.c_str(), item, length);
        _preferences.end();
        return OK;
    }
#else
    (void)key;
    (void)item;
    (void)length;
    (void)defaults;
#endif
    return ERROR_NOT_WRITTEN;
}

int32_t NonVolatileStorage::storeItem(const char* key, uint8_t pidProfileIndex, const void* item, size_t length, const void* defaults)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string itemKey = key + ('0' + pidProfileIndex);
        if (!memcmp(defaults, item, length)) {
            // value is the same as default, so no need to store it
            _preferences.remove(itemKey.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(itemKey.c_str(), item, length);
        _preferences.end();
        return OK;
    }
#else
    (void)key;
    (void)pidProfileIndex;
    (void)item;
    (void)length;
    (void)defaults;
#endif
    return ERROR_NOT_WRITTEN;
}
#endif //USE_FLASH_KLV

DynamicIdleController::config_t NonVolatileStorage::loadDynamicIdleControllerConfig(uint8_t pidProfileIndex) const
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return DEFAULTS::dynamicIdleControllerConfig;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = DynamicIdleControllerConfigKey + pidProfileIndex;
    DynamicIdleController::config_t config {};
    if (FlashKLV::OK == _flashKLV.read(&config, sizeof(config), key)) {
        return config;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    DynamicIdleController::config_t config {};
    if (loadItem(DynamicIdleControllerConfigKey, pidProfileIndex, &config, sizeof(config))) {
        return config;
    }
#endif
    return DEFAULTS::dynamicIdleControllerConfig;
}

int32_t NonVolatileStorage::storeDynamicIdleControllerConfig(const DynamicIdleController::config_t& config, uint8_t pidProfileIndex)
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = DynamicIdleControllerConfigKey + pidProfileIndex;
    if (!memcmp(&DEFAULTS::dynamicIdleControllerConfig, &config, sizeof(config))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(config), &config);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    return storeItem(DynamicIdleControllerConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::dynamicIdleControllerConfig);
#else
    (void)config;
    return OK;
#endif
}

FlightController::filters_config_t NonVolatileStorage::loadFlightControllerFiltersConfig(uint8_t pidProfileIndex) const
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return DEFAULTS::flightControllerFiltersConfig;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = FlightControllerFiltersConfigKey + pidProfileIndex;
    FlightController::filters_config_t config {};
    if (FlashKLV::OK == _flashKLV.read(&config, sizeof(config), key)) {
        return config;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    FlightController::filters_config_t config {};
    if (loadItem(DynamicIdleControllerConfigKey, pidProfileIndex, &config, sizeof(config))) {
        return config;
    }
#endif
    return DEFAULTS::flightControllerFiltersConfig;
}

int32_t NonVolatileStorage::storeFlightControllerFiltersConfig(const FlightController::filters_config_t& config, uint8_t pidProfileIndex)
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = FlightControllerFiltersConfigKey + pidProfileIndex;
    if (!memcmp(&DEFAULTS::flightControllerFiltersConfig, &config, sizeof(config))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(config), &config);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    return storeItem(FlightControllerFiltersConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerFiltersConfig);
#else
    (void)config;
    return OK;
#endif
}

FlightController::anti_gravity_config_t NonVolatileStorage::loadFlightControllerAntiGravityConfig(uint8_t pidProfileIndex) const
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return DEFAULTS::flightControllerAntiGravityConfig;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = FlightControllerAntiGravityConfigKey + pidProfileIndex;
    FlightController::anti_gravity_config_t config {};
    if (FlashKLV::OK == _flashKLV.read(&config, sizeof(config), key)) {
        return config;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    FlightController::anti_gravity_config_t config {};
    if (loadItem(DynamicIdleControllerConfigKey, pidProfileIndex, &config, sizeof(config))) {
        return config;
    }
#endif
    return DEFAULTS::flightControllerAntiGravityConfig;
}

int32_t NonVolatileStorage::storeFlightControllerAntiGravityConfig(const FlightController::anti_gravity_config_t& config, uint8_t pidProfileIndex)
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = FlightControllerAntiGravityConfigKey + pidProfileIndex;
    if (!memcmp(&DEFAULTS::flightControllerAntiGravityConfig, &config, sizeof(config))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(config), &config);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    return storeItem(FlightControllerAntiGravityConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerAntiGravityConfig);
#else
    (void)config;
    return OK;
#endif
}

FlightController::d_max_config_t NonVolatileStorage::loadFlightControllerDMaxConfig(uint8_t pidProfileIndex) const
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return DEFAULTS::flightControllerDMaxConfig;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = FlightControllerDMaxConfigKey + pidProfileIndex;
    FlightController::d_max_config_t config {};
    if (FlashKLV::OK == _flashKLV.read(&config, sizeof(config), key)) {
        return config;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    FlightController::d_max_config_t config {};
    if (loadItem(DynamicIdleControllerConfigKey, pidProfileIndex, &config, sizeof(config))) {
        return config;
    }
#endif
    return DEFAULTS::flightControllerDMaxConfig;
}

int32_t NonVolatileStorage::storeFlightControllerDMaxConfig(const FlightController::d_max_config_t& config, uint8_t pidProfileIndex)
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = FlightControllerDMaxConfigKey + pidProfileIndex;
    if (!memcmp(&DEFAULTS::flightControllerDMaxConfig, &config, sizeof(config))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(config), &config);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    return storeItem(FlightControllerDMaxConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerDMaxConfig);
#else
    (void)config;
    return OK;
#endif
}

IMU_Filters::config_t NonVolatileStorage::loadIMU_FiltersConfig() const
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = IMU_FiltersConfigKey;
    IMU_Filters::config_t config {};
    if (FlashKLV::OK == _flashKLV.read(&config, sizeof(config), key)) {
        return config;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    IMU_Filters::config_t config {};
    if (loadItem(DynamicIdleControllerConfigKey, &config, sizeof(config))) {
        return config;
    }
#endif
    return DEFAULTS::imuFiltersConfig;
}

int32_t NonVolatileStorage::storeIMU_FiltersConfig(const IMU_Filters::config_t& config)
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = IMU_FiltersConfigKey;
    if (!memcmp(&DEFAULTS::imuFiltersConfig, &config, sizeof(config))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(config), &config);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    return storeItem(IMU_FiltersConfigKey, &config, sizeof(config), &DEFAULTS::imuFiltersConfig);
#else
    (void)config;
    return OK;
#endif
}

RPM_Filters::config_t NonVolatileStorage::loadRPM_FiltersConfig() const
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = RPM_FiltersConfigKey;
    RPM_Filters::config_t config {};
    if (FlashKLV::OK == _flashKLV.read(&config, sizeof(config), key)) {
        return config;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    RPM_Filters::config_t config {};
    if (loadItem(DynamicIdleControllerConfigKey, &config, sizeof(config))) {
        return config;
    }
#endif
    return DEFAULTS::rpmFiltersConfig;
}

int32_t NonVolatileStorage::storeRPM_FiltersConfig(const RPM_Filters::config_t& config)
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = RPM_FiltersConfigKey;
    if (!memcmp(&DEFAULTS::rpmFiltersConfig, &config, sizeof(config))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(config), &config);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    return storeItem(RPM_FiltersConfigKey, &config, sizeof(config), &DEFAULTS::imuFiltersConfig);
#else
    (void)config;
    return OK;
#endif
}

RadioController::failsafe_t NonVolatileStorage::loadRadioControllerFailsafe() // NOLINT(readability-make-member-function-const)
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = RadioControllerFailsafeKey;
    RadioController::failsafe_t failsafe {};
    if (FlashKLV::OK ==  _flashKLV.read(&failsafe, sizeof(failsafe), key)) {
        return failsafe;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    RadioController::failsafe_t failsafe {};
    if (loadItem(DynamicIdleControllerConfigKey, &failsafe, sizeof(failsafe))) {
        return failsafe;
    }
#endif
    return DEFAULTS::radioControllerFailsafe;
}

int32_t NonVolatileStorage::storeRadioControllerFailsafe(const RadioController::failsafe_t& failsafe)
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = RadioControllerFailsafeKey;
    if (!memcmp(&DEFAULTS::radioControllerFailsafe, &failsafe, sizeof(failsafe))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(failsafe), &failsafe);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    return storeItem(RadioControllerFailsafeKey, &failsafe, sizeof(failsafe), &DEFAULTS::radioControllerFailsafe);
#else
    (void)failsafe;
    return OK;
#endif
}

RadioController::rates_t NonVolatileStorage::loadRadioControllerRates(uint8_t rateProfileIndex) const
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = RadioControllerRatesKey + rateProfileIndex;
    RadioController::rates_t rates {};
    if (FlashKLV::OK == _flashKLV.read(&rates, sizeof(rates), key)) {
        return rates;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = RadioControllerRatesKey + ('0' + rateProfileIndex);
        if (_preferences.isKey(key.c_str())) {
            RadioController::rates_t rates {};
            _preferences.getBytes(key.c_str(), &rates, sizeof(rates));
            _preferences.end();
            return rates;
        }
        _preferences.end();
    }
#else
    (void)rateProfileIndex;
#endif
    return DEFAULTS::radioControllerRates;
}

int32_t NonVolatileStorage::storeRadioControllerRates(const RadioController::rates_t& rates, uint8_t rateProfileIndex)
{
    if (rateProfileIndex >= RATE_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = RadioControllerRatesKey + rateProfileIndex;
    if (!memcmp(&DEFAULTS::radioControllerRates, &rates, sizeof(rates))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(rates), &rates);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = RadioControllerRatesKey + ('0' + rateProfileIndex);
        if (!memcmp(&DEFAULTS::radioControllerRates, &rates, sizeof(rates))) {
            // value is the same as default, so no need to store it
            _preferences.remove(key.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(key.c_str(), &rates, sizeof(rates));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)rateProfileIndex;
    (void)rates;
    return OK;
#endif
}

VehicleControllerBase::PIDF_uint16_t NonVolatileStorage::loadPID(uint8_t pidIndex, uint8_t pidProfileIndex) const
{
    assert(pidIndex <= FlightController::PID_COUNT);
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return DEFAULTS::flightControllerDefaultPIDs[pidIndex];
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = PID_Keys[pidIndex];
    VehicleControllerBase::PIDF_uint16_t pid {};
    if (FlashKLV::OK == _flashKLV.read(&pid, sizeof(pid), key)) {
        return pid;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = PID_Keys[pidIndex];
        if (_preferences.isKey(key.c_str())) {
            VehicleControllerBase::PIDF_uint16_t pid {};
            _preferences.getBytes(key.c_str(), &pid, sizeof(pid));
            _preferences.end();
            return pid;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::flightControllerDefaultPIDs[pidIndex];
}

int32_t NonVolatileStorage::storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex, uint8_t pidProfileIndex)
{
    assert(pidIndex <= FlightController::PID_COUNT);
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = PID_Keys[pidIndex];
    if (!memcmp(&DEFAULTS::flightControllerDefaultPIDs[pidIndex], &pid, sizeof(pid))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(pid), &pid);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = PID_Keys[pidIndex];
        if (!memcmp(&DEFAULTS::flightControllerDefaultPIDs[pidIndex], &pid, sizeof(pid))) {
            // value is the same as default, so no need to store it
            _preferences.remove(key.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(key.c_str(), &pid, sizeof(pid));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)pid;
    return OK;
#endif
}

void NonVolatileStorage::resetPID(uint8_t pidIndex, uint8_t pidProfileIndex)
{
    assert(pidIndex <= FlightController::PID_COUNT);
    assert(pidProfileIndex < PID_PROFILE_COUNT);
#if defined(USE_FLASH_KLV)
    _flashKLV.remove(PID_Keys[pidIndex]);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.remove(PID_Keys[pidIndex].c_str());
        _preferences.end();
    }
#endif
}


bool NonVolatileStorage::loadAccOffset(int32_t& x, int32_t& y, int32_t& z) const
{
#if defined(USE_FLASH_KLV)
    xyz_int32_t xyz {}; // NOLINT(misc-const-correctness)
    if (FlashKLV::OK == _flashKLV.read(&xyz, sizeof(xyz), AccOffsetKey)) {
        x = xyz.x;
        y = xyz.y;
        z = xyz.z;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        if (_preferences.isKey(AccOffsetKey)) {
            xyz_int32_t xyz {}; // NOLINT(misc-const-correctness) false positive
            _preferences.getBytes(AccOffsetKey, &xyz, sizeof(xyz));
            x = xyz.x;
            y = xyz.y;
            z = xyz.z;
        }
        _preferences.end();
        return true;
    }
#else
    (void)x;
    (void)y;
    (void)z;
#endif
    return false;
}

int32_t NonVolatileStorage::storeAccOffset(int32_t x, int32_t y, int32_t z)
{
#if defined(USE_FLASH_KLV)
    const xyz_int32_t xyz = { .x = x, .y = y, .z = z };
    return _flashKLV.write(AccOffsetKey, sizeof(xyz), &xyz);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const xyz_int32_t xyz = { .x = x, .y = y, .z = z };
        _preferences.putBytes(AccOffsetKey, &xyz, sizeof(xyz));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)x;
    (void)y;
    (void)z;
    return OK;
#endif
}

bool NonVolatileStorage::loadGyroOffset(int32_t& x, int32_t& y, int32_t& z) const
{
#if defined(USE_FLASH_KLV)
    xyz_int32_t xyz {}; // NOLINT(misc-const-correctness)
    if (FlashKLV::OK == _flashKLV.read(&xyz, sizeof(xyz), GyroOffsetKey)) {
        x = xyz.x;
        y = xyz.y;
        z = xyz.z;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        if (_preferences.isKey(GyroOffsetKey)) {
            xyz_int32_t xyz {}; // NOLINT(misc-const-correctness) false positive
            _preferences.getBytes(GyroOffsetKey, &xyz, sizeof(xyz));
            x = xyz.x;
            y = xyz.y;
            z = xyz.z;
        }
        _preferences.end();
        return true;
    }
#else
    (void)x;
    (void)y;
    (void)z;
#endif
    return false;
}

int32_t NonVolatileStorage::storeGyroOffset(int32_t x, int32_t y, int32_t z)
{
#if defined(USE_FLASH_KLV)
    const xyz_int32_t xyz = { .x = x, .y = y, .z = z };
    return _flashKLV.write(GyroOffsetKey, sizeof(xyz), &xyz);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const xyz_int32_t xyz = { .x = x, .y = y, .z = z };
        _preferences.putBytes(GyroOffsetKey, &xyz, sizeof(xyz));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)x;
    (void)y;
    (void)z;
    return OK;
#endif
}

void NonVolatileStorage::loadMacAddress(uint8_t* macAddress) const // NOLINT(readability-non-const-parameter)
{
#if defined(USE_FLASH_KLV)
    (void)macAddress;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        _preferences.getBytes(MacAddressKey, macAddress, MAC_ADDRESS_LEN);
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
        _preferences.putBytes(MacAddressKey, macAddress, MAC_ADDRESS_LEN);
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
    const AHRS& ahrs = flightController.getAHRS();
    const ReceiverBase& receiver = radioController.getReceiver();
    (void)receiver;

    const DynamicIdleController* dynamicIdleController = flightController.getMixer().getDynamicIdleController();
    if (dynamicIdleController) {
        const DynamicIdleController::config_t dynamicIdleControllerConfig = dynamicIdleController->getConfig();
        storeDynamicIdleControllerConfig(dynamicIdleControllerConfig, pidProfile);
    }

    const FlightController::filters_config_t flightControllerFiltersConfig = flightController.getFiltersConfig();
    storeFlightControllerFiltersConfig(flightControllerFiltersConfig, pidProfile);

    const FlightController::anti_gravity_config_t flightControllerAntiGravityConfig = flightController.getAntiGravityConfig();
    storeFlightControllerAntiGravityConfig(flightControllerAntiGravityConfig, pidProfile);

    const FlightController::d_max_config_t flightControllerDMaxConfig = flightController.getDMaxConfig();
    storeFlightControllerDMaxConfig(flightControllerDMaxConfig, pidProfile);

    const IMU_Filters::config_t imuFiltersConfig = static_cast<IMU_Filters&>(ahrs.getIMU_Filters()).getConfig(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    storeIMU_FiltersConfig(imuFiltersConfig);

    const RPM_Filters* rpmFilters = static_cast<IMU_Filters&>(ahrs.getIMU_Filters()).getRPM_Filters(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    if (rpmFilters) {
        const RPM_Filters::config_t rpmFiltersConfig = rpmFilters->getConfig(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
        storeRPM_FiltersConfig(rpmFiltersConfig);
    }

    const RadioController::rates_t& radioControllerRates = radioController.getRates();
    storeRadioControllerRates(radioControllerRates, ratesProfile);

    return OK;
}
