#include"NonVolatileStorage.h"
#include <AHRS.h>
#include <cstring>


#if defined(USE_FLASH_KLV)
// for FlashKLV, keys must be in range [0x0100, 0x3FFF]
static constexpr uint16_t PidProfileIndexKey = 0x0001;
static constexpr uint16_t RateProfileIndexKey = 0x0002;
static const std::array<uint16_t, FlightController::PID_COUNT> PID_Keys = {
    0x0100, 0x0101, 0x0102, 0x0103, 0x0104, 0x0105, 0x0106
};
static constexpr uint16_t AccOffsetKey = 0x0200;
static constexpr uint16_t GyroOffsetKey = 0x0201;
static constexpr uint16_t MacAddressKey = 0x0202;
// Part of PID profile
static constexpr uint16_t DynamicIdleControllerConfigKey = 0x0400;
static constexpr uint16_t FlightControllerFiltersConfigKey = 0x0404;
static constexpr uint16_t FlightControllerAntiGravityConfigKey = 0x0408;
static constexpr uint16_t FlightControllerDMaxConfigKey = 0x020C;

static constexpr uint16_t ImuFiltersConfigKey = 0x0207;
static constexpr uint16_t RadioControllerFailsafeKey = 0x0208;
static constexpr uint16_t RadioControllerRatesKey = 0x0209;

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

static const char* PidProfileIndexKey = "PPI";
static const char* RateProfileIndexKey = "RPI";
static const char* DynamicIdleControllerConfigKey = "DIC";
static const char* FlightControllerFiltersConfigKey = "FCF";
static const char* FlightControllerAntiGravityConfigKey = "FCAG";
static const char* FlightControllerDMaxConfigKey = "FCDM";
static const char* ImuFiltersConfigKey = "IF";
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

uint8_t NonVolatileStorage::PidProfileIndexLoad() const
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = PidProfileIndexKey;
    uint8_t profileIndex {}; // NOLINT(misc-const-correctness) false positive
    if (FlashKLV::OK == _flashKLV.read(&profileIndex, sizeof(profileIndex), key)) {
        return profileIndex;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = PidProfileIndexKey;
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

int32_t NonVolatileStorage::PidProfileIndexStore(uint8_t profileIndex)
{
    if (profileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = PidProfileIndexKey;
    if (profileIndex == DEFAULT_PID_PROFILE) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(profileIndex), &profileIndex);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = PidProfileIndexKey;
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

uint8_t NonVolatileStorage::RateProfileIndexLoad() const
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

int32_t NonVolatileStorage::RateProfileIndexStore(uint8_t profileIndex)
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

DynamicIdleController::config_t NonVolatileStorage::DynamicIdleControllerConfigLoad(uint8_t pidProfileIndex) const
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
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = DynamicIdleControllerConfigKey + ('0' + pidProfileIndex);
        if (_preferences.isKey(key.c_str())) {
            DynamicIdleController::config_t config {};
            _preferences.getBytes(key.c_str(), &config, sizeof(config));
            _preferences.end();
            return config;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::dynamicIdleControllerConfig;
}

int32_t NonVolatileStorage::DynamicIdleControllerConfigStore(const DynamicIdleController::config_t& config, uint8_t pidProfileIndex)
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
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = DynamicIdleControllerConfigKey + ('0' + pidProfileIndex);
        if (!memcmp(&DEFAULTS::dynamicIdleControllerConfig, &config, sizeof(config))) {
            // value is the same as default, so no need to store it
            _preferences.remove(key.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(key.c_str(), &config, sizeof(config));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)config;
    return OK;
#endif
}

FlightController::filters_config_t NonVolatileStorage::FlightControllerFiltersConfigLoad(uint8_t pidProfileIndex) const
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
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = FlightControllerFiltersConfigKey + ('0' + pidProfileIndex);
        if (_preferences.isKey(key.c_str())) {
            FlightController::filters_config_t config {};
            _preferences.getBytes(key.c_str(), &config, sizeof(config));
            _preferences.end();
            return config;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::flightControllerFiltersConfig;
}

int32_t NonVolatileStorage::FlightControllerFiltersConfigStore(const FlightController::filters_config_t& config, uint8_t pidProfileIndex)
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
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = FlightControllerFiltersConfigKey + ('0' + pidProfileIndex);
        if (!memcmp(&DEFAULTS::flightControllerFiltersConfig, &config, sizeof(config))) {
            // value is the same as default, so no need to store it
            _preferences.remove(key.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(key.c_str(), &config, sizeof(config));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)config;
    return OK;
#endif
}

FlightController::anti_gravity_config_t NonVolatileStorage::FlightControllerAntiGravityConfigLoad(uint8_t pidProfileIndex) const
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
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = FlightControllerAntiGravityConfigKey + ('0' + pidProfileIndex);
        if (_preferences.isKey(key.c_str())) {
            FlightController::anti_gravity_config_t config {};
            _preferences.getBytes(key.c_str(), &config, sizeof(config));
            _preferences.end();
            return config;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::flightControllerAntiGravityConfig;
}

int32_t NonVolatileStorage::FlightControllerAntiGravityConfigStore(const FlightController::anti_gravity_config_t& config, uint8_t pidProfileIndex)
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
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = FlightControllerAntiGravityConfigKey + ('0' + pidProfileIndex);
        if (!memcmp(&DEFAULTS::flightControllerDMaxConfig, &config, sizeof(config))) {
            // value is the same as default, so no need to store it
            _preferences.remove(key.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(key.c_str(), &config, sizeof(config));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)config;
    return OK;
#endif
}

FlightController::d_max_config_t NonVolatileStorage::FlightControllerDMaxConfigLoad(uint8_t pidProfileIndex) const
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
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = FlightControllerDMaxConfigKey + ('0' + pidProfileIndex);
        if (_preferences.isKey(key.c_str())) {
            FlightController::d_max_config_t config {};
            _preferences.getBytes(key.c_str(), &config, sizeof(config));
            _preferences.end();
            return config;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::flightControllerDMaxConfig;
}

int32_t NonVolatileStorage::FlightControllerDMaxConfigStore(const FlightController::d_max_config_t& config, uint8_t pidProfileIndex)
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
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = FlightControllerDMaxConfigKey + ('0' + pidProfileIndex);
        if (!memcmp(&DEFAULTS::flightControllerDMaxConfig, &config, sizeof(config))) {
            // value is the same as default, so no need to store it
            _preferences.remove(key.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(key.c_str(), &config, sizeof(config));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)config;
    return OK;
#endif
}

IMU_Filters::config_t NonVolatileStorage::ImuFiltersConfigLoad() const
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = ImuFiltersConfigKey;
    IMU_Filters::config_t config {};
    if (FlashKLV::OK == _flashKLV.read(&config, sizeof(config), key)) {
        return config;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = ImuFiltersConfigKey;
        if (_preferences.isKey(key.c_str())) {
            IMU_Filters::config_t config {};
            _preferences.getBytes(key.c_str(), &config, sizeof(config));
            _preferences.end();
            return config;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::imuFiltersConfig;
}

int32_t NonVolatileStorage::ImuFiltersConfigStore(const IMU_Filters::config_t& config)
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = ImuFiltersConfigKey;
    if (!memcmp(&DEFAULTS::imuFiltersConfig, &config, sizeof(config))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, sizeof(config), &config);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = ImuFiltersConfigKey;
        if (!memcmp(&DEFAULTS::imuFiltersConfig, &config, sizeof(config))) {
            // value is the same as default, so no need to store it
            _preferences.remove(key.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(key.c_str(), &config, sizeof(config));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)config;
    return OK;
#endif
}

RadioController::failsafe_t NonVolatileStorage::RadioControllerFailsafeLoad()
{
#if defined(USE_FLASH_KLV)
    const uint16_t key = RadioControllerFailsafeKey;
    RadioController::failsafe_t failsafe {};
    if (FlashKLV::OK ==  _flashKLV.read(&failsafe, sizeof(failsafe), key)) {
        return failsafe;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        const std::string key = RadioControllerFailsafeKey;
        if (_preferences.isKey(key.c_str())) {
            RadioController::failsafe_t failsafe {};
            _preferences.getBytes(key.c_str(), &failsafe, sizeof(failsafe));
            _preferences.end();
            return failsafe;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::radioControllerFailsafe;
}

int32_t NonVolatileStorage::RadioControllerFailsafeStore(const RadioController::failsafe_t& failsafe)
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
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const std::string key = RadioControllerFailsafeKey;
        if (!memcmp(&DEFAULTS::radioControllerFailsafe, &failsafe, sizeof(failsafe))) {
            // value is the same as default, so no need to store it
            _preferences.remove(key.c_str());
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(key.c_str(), &failsafe, sizeof(failsafe));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)failsafe;
    return OK;
#endif
}

RadioController::rates_t NonVolatileStorage::RadioControllerRatesLoad(uint8_t rateProfileIndex) const
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

int32_t NonVolatileStorage::RadioControllerRatesStore(const RadioController::rates_t& rates, uint8_t rateProfileIndex)
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

VehicleControllerBase::PIDF_uint16_t NonVolatileStorage::PID_load(uint8_t pidIndex, uint8_t pidProfileIndex) const
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

int32_t NonVolatileStorage::PID_store(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex, uint8_t pidProfileIndex)
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
    (void)pidIndex;
    (void)pid;
    return OK;
#endif
}

void NonVolatileStorage::PID_reset(uint8_t pidIndex)
{
    assert(pidIndex <= FlightController::PID_COUNT);
#if defined(USE_FLASH_KLV)
    _flashKLV.remove(PID_Keys[pidIndex]);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.remove(PID_Keys[pidIndex].c_str());
        _preferences.end();
    }
#else
    (void)pidIndex;
#endif
}


bool NonVolatileStorage::AccOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const
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

int32_t NonVolatileStorage::AccOffsetStore(int32_t x, int32_t y, int32_t z)
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

bool NonVolatileStorage::GyroOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const
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

int32_t NonVolatileStorage::GyroOffsetStore(int32_t x, int32_t y, int32_t z)
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

void NonVolatileStorage::MacAddressLoad(uint8_t* macAddress) const // NOLINT(readability-non-const-parameter)
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

int32_t NonVolatileStorage::MacAddressStore(const uint8_t* macAddress)
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

int32_t NonVolatileStorage::storeAll(const AHRS& ahrs, const FlightController& flightController, const RadioController& radioController, const ReceiverBase& receiver, uint8_t pidProfile, uint8_t ratesProfile)
{
    (void)receiver;

    const DynamicIdleController* dynamicIdleController = flightController.getMixer().getDynamicIdleController();
    if (dynamicIdleController) {
        const DynamicIdleController::config_t dynamicIdleControllerConfig = dynamicIdleController->getConfig();
        DynamicIdleControllerConfigStore(dynamicIdleControllerConfig, pidProfile);
    }

    const FlightController::filters_config_t flightControllerFiltersConfig = flightController.getFiltersConfig();
    FlightControllerFiltersConfigStore(flightControllerFiltersConfig, pidProfile);

    const FlightController::anti_gravity_config_t flightControllerAntiGravityConfig = flightController.getAntiGravityConfig();
    FlightControllerAntiGravityConfigStore(flightControllerAntiGravityConfig, pidProfile);

    const FlightController::d_max_config_t flightControllerDMaxConfig = flightController.getDMaxConfig();
    FlightControllerDMaxConfigStore(flightControllerDMaxConfig, pidProfile);

    const IMU_Filters::config_t imuFiltersConfig = static_cast<IMU_Filters&>(ahrs.getIMU_Filters()).getConfig(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    ImuFiltersConfigStore(imuFiltersConfig);

    const RadioController::rates_t& radioControllerRates = radioController.getRates();
    RadioControllerRatesStore(radioControllerRates, ratesProfile);

    return OK;
}
