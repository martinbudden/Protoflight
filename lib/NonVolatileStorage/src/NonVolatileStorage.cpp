#include"NonVolatileStorage.h"
#include <AHRS.h>


#if defined(USE_ARDUINO_ESP32_PREFERENCES)
static const char* nonVolatileStorageNamespace {"PTFL"}; // ProtoFlight
#endif

const std::array<std::string, FlightController::PID_COUNT> NonVolatileStorage::PID_Keys = {
    "ROLL_RATE",
    "PITCH_RATE",
    "YAW_RATE",
    "ROLL_ANGLE",
    "PITCH_ANGLE"
    "ROLL_SIN_ANGLE",
    "PITCH_SIN_ANGLE"
};

const char* NonVolatileStorage::FlightControllerFiltersConfigKey = "FCF";
const char* NonVolatileStorage::ImuFiltersConfigKey = "IF";
const char* NonVolatileStorage::DynamicIdleControllerConfigKey = "DIC";
const char* NonVolatileStorage::RadioControllerRatesKey = "RCR";
const char* NonVolatileStorage::AccOffsetKey = "ACC";
const char* NonVolatileStorage::GyroOffsetKey = "GYR";
const char* NonVolatileStorage::MacAddressKey = "MAC";

void NonVolatileStorage::init()
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        if (!_preferences.getBool("init")) {
            _preferences.putBool("init", true);
        }
        _preferences.end();
    }
#endif
}

void NonVolatileStorage::clear()
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.clear();
        _preferences.end();
    }
#endif
}

void NonVolatileStorage::remove(const std::string& name)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.remove(name.c_str());
    }
#else
    (void)name;
#endif
}

void NonVolatileStorage::storeAll(const AHRS& ahrs, const FlightController& flightController, const RadioController& radioController, const ReceiverBase& receiver)
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
}

DynamicIdleController::config_t NonVolatileStorage::DynamicIdleControllerConfigLoad() const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
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

void  NonVolatileStorage::DynamicIdleControllerConfigStore(const DynamicIdleController::config_t& config)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(DynamicIdleControllerConfigKey, &config, sizeof(config));
        _preferences.end();
    }
#else
    (void)config;
#endif
}

FlightController::filters_config_t NonVolatileStorage::FlightControllerFiltersConfigLoad() const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        if (_preferences.isKey(ImuFiltersConfigKey)) {
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

void NonVolatileStorage::FlightControllerFiltersConfigStore(const FlightController::filters_config_t& config)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(FlightControllerFiltersConfigKey, &config, sizeof(config));
        _preferences.end();
    }
#else
    (void)config;
#endif
}

IMU_Filters::config_t NonVolatileStorage::ImuFiltersConfigLoad() const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
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

void NonVolatileStorage::ImuFiltersConfigStore(const IMU_Filters::config_t& config)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(ImuFiltersConfigKey, &config, sizeof(config));
        _preferences.end();
    }
#else
    (void)config;
#endif
}

RadioController::rates_t NonVolatileStorage::RadioControllerRatesLoad() const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
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

void NonVolatileStorage::RadioControllerRatesStore(const RadioController::rates_t& rates)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(RadioControllerRatesKey, &rates, sizeof(rates));
        _preferences.end();
    }
#else
    (void)rates;
#endif
}

PIDF::PIDF_t NonVolatileStorage::PID_load(uint8_t index) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
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

void NonVolatileStorage::PID_store(uint8_t index, const PIDF::PIDF_t& pid)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(PID_Keys[index].c_str(), &pid, sizeof(pid));
        _preferences.end();
    }
#else
    (void)index;
    (void)pid;
#endif
}

bool NonVolatileStorage::AccOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
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

void NonVolatileStorage::AccOffsetStore(int32_t x, int32_t y, int32_t z)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const xyz_int32_t xyz = { .x = x, .y = y, .z = z };
        _preferences.putBytes(AccOffsetKey, &xyz, sizeof(xyz));
        _preferences.end();
    }
#else
    (void)x;
    (void)y;
    (void)z;
#endif
}

bool NonVolatileStorage::GyroOffsetLoad(int32_t& x, int32_t& y, int32_t& z) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
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

void NonVolatileStorage::GyroOffsetStore(int32_t x, int32_t y, int32_t z)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        const xyz_int32_t xyz = { .x = x, .y = y, .z = z };
        _preferences.putBytes(GyroOffsetKey, &xyz, sizeof(xyz));
        _preferences.end();
    }
#else
    (void)x;
    (void)y;
    (void)z;
#endif
}

void NonVolatileStorage::MacAddressLoad(uint8_t* macAddress) const // NOLINT(readability-non-const-parameter)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        _preferences.getBytes(MacAddressKey, macAddress, MAC_ADDRESS_LEN);
        _preferences.end();
    }
#else
    (void)macAddress;
#endif
}

void NonVolatileStorage::MacAddressStore(const uint8_t* macAddress)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.putBytes(MacAddressKey, macAddress, MAC_ADDRESS_LEN);
        _preferences.end();
    }
#else
    (void)macAddress;
#endif
}
