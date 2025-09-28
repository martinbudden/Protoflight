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

#if defined(USE_FLASH_KLV)

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
static constexpr uint16_t FlightControllerTPA_ConfigKey = 0x408;
static constexpr uint16_t FlightControllerAntiGravityConfigKey = 0x040C;
static constexpr uint16_t FlightControllerDMaxConfigKey = 0x0410;

static constexpr uint16_t RadioControllerRatesKey = 0x0500; // note jump of 4 to allow storage of 4 rates profiles
static constexpr uint16_t IMU_FiltersConfigKey = 0x0504;
static constexpr uint16_t RPM_FiltersConfigKey = 0x0505;
static constexpr uint16_t RadioControllerFailsafeKey = 0x0506;

#else

// NOTE: the ESP32 Preferences versions take advantage of the Small String Optimization (SSO)
// which means that strings of less than (about) 16 characters are stored in the string object
// rather than on the heap.
static const std::array<std::string, FlightController::PID_COUNT> PID_Keys = {
    "RoRa",
    "PiRa",
    "YaRa",
    "RoAn",
    "PiAn"
    "RoSA",
    "PiSA"
};

#if defined(USE_ARDUINO_ESP32_PREFERENCES)
static const char* nonVolatileStorageNamespace {"PTFL"}; // ProtoFlight
static const char* MacAddressKey = "MAC";
#endif

static const char* DynamicIdleControllerConfigKey = "DIC";
static const char* FlightControllerFiltersConfigKey = "FCF";
static const char* FlightControllerTPA_ConfigKey = "FCTPA";
static const char* FlightControllerAntiGravityConfigKey = "FCAG";
static const char* FlightControllerDMaxConfigKey = "FCDM";
static const char* IMU_FiltersConfigKey = "IF";
static const char* RPM_FiltersConfigKey = "RPM";
static const char* RadioControllerFailsafeKey = "RCFS";
static const char* RadioControllerRatesKey = "RCR";

static const char* PID_ProfileIndexKey = "PPI";
static const char* RateProfileIndexKey = "RPI";

static const char* AccOffsetKey = "ACC";
static const char* GyroOffsetKey = "GYR";

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

#if defined(USE_FLASH_KLV)
bool NonVolatileStorage::loadItem(uint16_t key, void* item, size_t length) const
{
    if (FlashKLV::OK == _flashKLV.read(item, length, key)) {
        return true;
    }
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
    if (!memcmp(defaults, item, length)) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, length, item);
}

int32_t NonVolatileStorage::storeItem(uint16_t key, uint8_t pidProfileIndex, const void* item, size_t length, const void* defaults)
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    return storeItem(key + pidProfileIndex, item, length, defaults);
}

#else

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

bool NonVolatileStorage::loadItem(const char* key, uint8_t pidProfileIndex, void* item, size_t length) const
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return false;
    }
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    const std::string itemKey = key + ('0' + pidProfileIndex);
    return loadItem(itemKey.c_str(), item, length);
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
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    const std::string itemKey = key + ('0' + pidProfileIndex);
    return storeItem(itemKey.c_str(), item, length, defaults);
#else
    (void)key;
    (void)item;
    (void)length;
    (void)defaults;
#endif
    return ERROR_NOT_WRITTEN;
}
#endif //USE_FLASH_KLV



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
    DynamicIdleController::config_t config {};
    if (loadItem(DynamicIdleControllerConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }
    return DEFAULTS::dynamicIdleControllerConfig;
}

int32_t NonVolatileStorage::storeDynamicIdleControllerConfig(const DynamicIdleController::config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(DynamicIdleControllerConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::dynamicIdleControllerConfig);
}


FlightController::filters_config_t NonVolatileStorage::loadFlightControllerFiltersConfig(uint8_t pidProfileIndex) const
{
    FlightController::filters_config_t config {};
    if (loadItem(FlightControllerFiltersConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }
    return DEFAULTS::flightControllerFiltersConfig;
}

int32_t NonVolatileStorage::storeFlightControllerFiltersConfig(const FlightController::filters_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerFiltersConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerFiltersConfig);
}


FlightController::tpa_config_t NonVolatileStorage::loadFlightControllerTPA_Config(uint8_t pidProfileIndex) const
{
    FlightController::tpa_config_t config {};
    if (loadItem(FlightControllerTPA_ConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }
    return DEFAULTS::flightControllerTPA_Config;
}

int32_t NonVolatileStorage::storeFlightControllerTPA_Config(const FlightController::tpa_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerTPA_ConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerTPA_Config);
}


FlightController::anti_gravity_config_t NonVolatileStorage::loadFlightControllerAntiGravityConfig(uint8_t pidProfileIndex) const
{
    FlightController::anti_gravity_config_t config {};
    if (loadItem(FlightControllerAntiGravityConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }
    return DEFAULTS::flightControllerAntiGravityConfig;
}

int32_t NonVolatileStorage::storeFlightControllerAntiGravityConfig(const FlightController::anti_gravity_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerAntiGravityConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerAntiGravityConfig);
}


FlightController::d_max_config_t NonVolatileStorage::loadFlightControllerDMaxConfig(uint8_t pidProfileIndex) const
{
    FlightController::d_max_config_t config {};
    if (loadItem(FlightControllerDMaxConfigKey, pidProfileIndex, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }
    return DEFAULTS::flightControllerDMaxConfig;
}

int32_t NonVolatileStorage::storeFlightControllerDMaxConfig(const FlightController::d_max_config_t& config, uint8_t pidProfileIndex)
{
    return storeItem(FlightControllerDMaxConfigKey, pidProfileIndex, &config, sizeof(config), &DEFAULTS::flightControllerDMaxConfig);
}


IMU_Filters::config_t NonVolatileStorage::loadIMU_FiltersConfig() const
{
    IMU_Filters::config_t config {};
    if (loadItem(IMU_FiltersConfigKey, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }
    return DEFAULTS::imuFiltersConfig;
}

int32_t NonVolatileStorage::storeIMU_FiltersConfig(const IMU_Filters::config_t& config)
{
    return storeItem(IMU_FiltersConfigKey, &config, sizeof(config), &DEFAULTS::imuFiltersConfig);
}


RPM_Filters::config_t NonVolatileStorage::loadRPM_FiltersConfig() const
{
    RPM_Filters::config_t config {};
    if (loadItem(RPM_FiltersConfigKey, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }
    return DEFAULTS::rpmFiltersConfig;
}

int32_t NonVolatileStorage::storeRPM_FiltersConfig(const RPM_Filters::config_t& config)
{
    return storeItem(RPM_FiltersConfigKey, &config, sizeof(config), &DEFAULTS::rpmFiltersConfig);
}


RadioController::failsafe_t NonVolatileStorage::loadRadioControllerFailsafe() // NOLINT(readability-make-member-function-const)
{
    RadioController::failsafe_t failsafe {};
    if (loadItem(RadioControllerFailsafeKey, &failsafe, sizeof(failsafe))) { // cppcheck-suppress knownConditionTrueFalse
    }
    return DEFAULTS::radioControllerFailsafe;
}

int32_t NonVolatileStorage::storeRadioControllerFailsafe(const RadioController::failsafe_t& failsafe)
{
    return storeItem(RadioControllerFailsafeKey, &failsafe, sizeof(failsafe), &DEFAULTS::radioControllerFailsafe);
}

RadioController::rates_t NonVolatileStorage::loadRadioControllerRates(uint8_t rateProfileIndex) const
{
    if (rateProfileIndex >= RATE_PROFILE_COUNT) {
        return DEFAULTS::radioControllerRates;
    }
    RadioController::rates_t rates {};
#if defined(USE_FLASH_KLV)
    const uint16_t key = RadioControllerRatesKey + rateProfileIndex;
#else
    const std::string keyString = RadioControllerRatesKey + ('0' + rateProfileIndex);
    const char* key = keyString.c_str();
#endif
    if (loadItem(key, &rates, sizeof(rates))) { // cppcheck-suppress knownConditionTrueFalse
        return rates;
    }
    return DEFAULTS::radioControllerRates;
}

int32_t NonVolatileStorage::storeRadioControllerRates(const RadioController::rates_t& rates, uint8_t rateProfileIndex)
{
    if (rateProfileIndex >= RATE_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = RadioControllerRatesKey + rateProfileIndex;
#else
    const std::string keyString = RadioControllerRatesKey + ('0' + rateProfileIndex);
    const char* key = keyString.c_str();
#endif
    return storeItem(key, &rates, sizeof(rates), &DEFAULTS::radioControllerRates);
}

VehicleControllerBase::PIDF_uint16_t NonVolatileStorage::loadPID(uint8_t pidIndex, uint8_t pidProfileIndex) const
{
    assert(pidIndex <= FlightController::PID_COUNT);
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return DEFAULTS::flightControllerDefaultPIDs[pidIndex];
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = PID_Keys[pidIndex] + pidProfileIndex;
#else
    const std::string keyString = PID_Keys[pidIndex] + static_cast<char>('0' + pidProfileIndex);
    const char* key = keyString.c_str();
#endif
    VehicleControllerBase::PIDF_uint16_t pid {};
    if (loadItem(key, &pid, sizeof(pid))) { // cppcheck-suppress knownConditionTrueFalse
        return pid;
    }
    return DEFAULTS::flightControllerDefaultPIDs[pidIndex];
}

int32_t NonVolatileStorage::storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex, uint8_t pidProfileIndex)
{
    assert(pidIndex <= FlightController::PID_COUNT);
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
#if defined(USE_FLASH_KLV)
    const uint16_t key = PID_Keys[pidIndex] + pidProfileIndex;
#else
    const std::string keyString = PID_Keys[pidIndex] + static_cast<char>('0' + pidProfileIndex);
    const char* key = keyString.c_str();
#endif
    return storeItem(key, &key, sizeof(pid), &DEFAULTS::flightControllerDefaultPIDs[pidIndex]);
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
