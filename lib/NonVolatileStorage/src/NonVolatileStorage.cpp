#include"NonVolatileStorage.h"
#include <AHRS.h>
#include <cstring>


#if defined(USE_FLASH_KLV)
// for FlashKLV, keys must be in range [0x0100, 0x3FFF]
    static const std::array<uint16_t, FlightController::PID_COUNT> PID_Keys = {
        0x0100, 0x0101, 0x0102, 0x0103, 0x0104, 0x0105, 0x0106
    };
    static constexpr uint16_t AccOffsetKey = 0x0200;
    static constexpr uint16_t GyroOffsetKey = 0x0201;
    static constexpr uint16_t MacAddressKey = 0x0202;
    static constexpr uint16_t DynamicIdleControllerConfigKey = 0x0203;
    static constexpr uint16_t FlightControllerFiltersConfigKey = 0x0204;
    static constexpr uint16_t ImuFiltersConfigKey = 0x0205;
    static constexpr uint16_t RadioControllerFailsafeKey = 0x0206;
    static constexpr uint16_t RadioControllerRatesKey = 0x0207;

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

static const char* DynamicIdleControllerConfigKey = "DIC";
static const char* FlightControllerFiltersConfigKey = "FCF";
static const char* ImuFiltersConfigKey = "IF";
static const char* RadioControllerFailsafeKey = "RCF";
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
    NonVolatileStorage(0)
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

int32_t NonVolatileStorage::storeAll(const AHRS& ahrs, const FlightController& flightController, const RadioController& radioController, const ReceiverBase& receiver)
{
    (void)receiver;

    const DynamicIdleController* dynamicIdleController = flightController.getMixer().getDynamicIdleController();
    if (dynamicIdleController) {
        const DynamicIdleController::config_t dynamicIdleControllerConfig = dynamicIdleController->getConfig();
        DynamicIdleControllerConfigStore(dynamicIdleControllerConfig);
    }

    const FlightController::filters_config_t flightControllerFiltersConfig = flightController.getFiltersConfig();
    FlightControllerFiltersConfigStore(flightControllerFiltersConfig);

    const IMU_Filters::config_t imuFiltersConfig = static_cast<IMU_Filters&>(ahrs.getIMU_Filters()).getConfig(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    ImuFiltersConfigStore(imuFiltersConfig);

    const RadioController::rates_t& radioControllerRates = radioController.getRates();
    RadioControllerRatesStore(radioControllerRates);

    return OK;
}

DynamicIdleController::config_t NonVolatileStorage::DynamicIdleControllerConfigLoad() const
{
#if defined(USE_FLASH_KLV)
    DynamicIdleController::config_t config {};
    if (FlashKLV::OK == _flashKLV.read(&config, sizeof(config), DynamicIdleControllerConfigKey)) {
        return config;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        if (_preferences.isKey(DynamicIdleControllerConfigKey)) {
            DynamicIdleController::config_t config {};
            //const size_t len = _preferences.getBytesLength("DIC");
            _preferences.getBytes(DynamicIdleControllerConfigKey, &config, sizeof(config));
            _preferences.end();
            return config;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::dynamicIdleControllerConfig;
}

int32_t NonVolatileStorage::DynamicIdleControllerConfigStore(const DynamicIdleController::config_t& config)
{
#if defined(USE_FLASH_KLV)
    if (!memcmp(&DEFAULTS::dynamicIdleControllerConfig, &config, sizeof(config))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(DynamicIdleControllerConfigKey);
        return OK_IS_DEFAULT;
    } else {
        return _flashKLV.write(DynamicIdleControllerConfigKey, sizeof(config), &config);
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(DynamicIdleControllerConfigKey, &config, sizeof(config));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)config;
    return OK;
#endif
}

FlightController::filters_config_t NonVolatileStorage::FlightControllerFiltersConfigLoad() const
{
#if defined(USE_FLASH_KLV)
    FlightController::filters_config_t config {};
    if (FlashKLV::OK == _flashKLV.read(&config, sizeof(config), FlightControllerFiltersConfigKey)) {
        return config;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        if (_preferences.isKey(FlightControllerFiltersConfigKey)) {
            FlightController::filters_config_t config {};
            //const size_t len = _preferences.getBytesLength("DIC");
            _preferences.getBytes(FlightControllerFiltersConfigKey, &config, sizeof(config));
            _preferences.end();
            return config;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::flightControllerFiltersConfig;
}

int32_t NonVolatileStorage::FlightControllerFiltersConfigStore(const FlightController::filters_config_t& config)
{
#if defined(USE_FLASH_KLV)
    if (!memcmp(&DEFAULTS::flightControllerFiltersConfig, &config, sizeof(config))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(FlightControllerFiltersConfigKey);
        return OK_IS_DEFAULT;
    } else {
        return _flashKLV.write(FlightControllerFiltersConfigKey, sizeof(config), &config);
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(FlightControllerFiltersConfigKey, &config, sizeof(config));
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
    IMU_Filters::config_t config {};
    if (FlashKLV::OK == _flashKLV.read(&config, sizeof(config), ImuFiltersConfigKey)) {
        return config;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        if (_preferences.isKey(ImuFiltersConfigKey)) {
            IMU_Filters::config_t config {};
            //const size_t len = _preferences.getBytesLength("DIC");
            _preferences.getBytes(ImuFiltersConfigKey, &config, sizeof(config));
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
    if (!memcmp(&DEFAULTS::imuFiltersConfig, &config, sizeof(config))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(ImuFiltersConfigKey);
        return OK_IS_DEFAULT;
    } else {
        return _flashKLV.write(ImuFiltersConfigKey, sizeof(config), &config);
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(ImuFiltersConfigKey, &config, sizeof(config));
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
    RadioController::failsafe_t failsafe {};
    if (FlashKLV::OK ==  _flashKLV.read(&failsafe, sizeof(failsafe), RadioControllerFailsafeKey)) {
        return failsafe;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        if (_preferences.isKey(RadioControllerFailsafeKey)) {
            RadioController::failsafe_t failsafe {};
            _preferences.getBytes(RadioControllerFailsafeKey, &failsafe, sizeof(failsafe));
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
    if (!memcmp(&DEFAULTS::radioControllerFailsafe, &failsafe, sizeof(failsafe))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(RadioControllerFailsafeKey);
        return OK_IS_DEFAULT;
    } else {
        return _flashKLV.write(RadioControllerFailsafeKey, sizeof(failsafe), &failsafe);
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(RadioControllerFailsafeKey, &failsafe, sizeof(failsafe));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)failsafe;
    return OK;
#endif
}

RadioController::rates_t NonVolatileStorage::RadioControllerRatesLoad() const
{
#if defined(USE_FLASH_KLV)
    RadioController::rates_t rates {};
    if (FlashKLV::OK == _flashKLV.read(&rates, sizeof(rates), RadioControllerRatesKey)) {
        return rates;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        if (_preferences.isKey(RadioControllerRatesKey)) {
            RadioController::rates_t rates {};
            _preferences.getBytes(RadioControllerRatesKey, &rates, sizeof(rates));
            _preferences.end();
            return rates;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::radioControllerRates;
}

int32_t NonVolatileStorage::RadioControllerRatesStore(const RadioController::rates_t& rates)
{
#if defined(USE_FLASH_KLV)
    if (!memcmp(&DEFAULTS::radioControllerRates, &rates, sizeof(rates))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(RadioControllerRatesKey);
        return OK_IS_DEFAULT;
    } else {
        return _flashKLV.write(RadioControllerRatesKey, sizeof(rates), &rates);
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(RadioControllerRatesKey, &rates, sizeof(rates));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)rates;
    return OK;
#endif
}

PIDF::PIDF_t NonVolatileStorage::PID_load(uint8_t index) const
{
#if defined(USE_FLASH_KLV)
    PIDF::PIDF_t pid {};
    if (FlashKLV::OK == _flashKLV.read(&pid, sizeof(pid), PID_Keys[index])) {
        return pid;
    }
#elif defined(USE_FLASH_KLV)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        if (_preferences.isKey(PID_Keys[index].c_str())) {
            PIDF::PIDF_t pid {};
            _preferences.getBytes(PID_Keys[index].c_str(), &pid, sizeof(pid));
            _preferences.end();
            return pid;
        }
        _preferences.end();
    }
#endif
    return DEFAULTS::flightControllerDefaultPIDs[index];
}

int32_t NonVolatileStorage::PID_store(uint8_t index, const PIDF::PIDF_t& pid)
{
#if defined(USE_FLASH_KLV)
    const uint32_t key = PID_Keys[index];
    if (!memcmp(&DEFAULTS::flightControllerDefaultPIDs[index], &pid, sizeof(pid))) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    } else {
        return _flashKLV.write(key, sizeof(pid), &pid);
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(PID_Keys[index].c_str(), &pid, sizeof(pid));
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)index;
    (void)pid;
    return OK;
#endif
}

bool NonVolatileStorage::AccOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const
{
#if defined(USE_FLASH_KLV)
    xyz_int32_t xyz {};
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
    xyz_int32_t xyz {};
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
