#include"NonVolatileStorage.h"


#if defined(USE_ARDUINO_ESP32_PREFERENCES)
static const char* nonVolatileStorageNamespace {"PTFL"}; // ProtoFlight
#endif

void NonVolatileStorage::clear()
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);

    _preferences.clear();

    _preferences.end();
#endif
}

IMU_Filters::config_t NonVolatileStorage::loadImuFiltersConfig() const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_ONLY);
    if (_preferences.isKey("IF")) {
        IMU_Filters::config_t config {};
        //const size_t len = _preferences.getBytesLength("DIC");
        _preferences.getBytes("IF", &config, sizeof(config));
        _preferences.end();
        return config;
    }
    _preferences.end();
#endif
    return DEFAULTS::imuFiltersConfig;
}

void NonVolatileStorage::storeImuFiltersConfig(const IMU_Filters::config_t& config)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);
    _preferences.putBytes("IF", &config, sizeof(config));
    _preferences.end();
#else
    (void)config;
#endif
}

DynamicIdleController::config_t NonVolatileStorage::loadDynamicIdleControllerConfig() const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_ONLY);
    if (_preferences.isKey("DIC")) {
        DynamicIdleController::config_t config {};
        //const size_t len = _preferences.getBytesLength("DIC");
        _preferences.getBytes("DIC", &config, sizeof(config));
        _preferences.end();
        return config;
    }
    _preferences.end();
#endif
    return DEFAULTS::dynamicIdleControllerConfig;
}

void  NonVolatileStorage::storeDynamicIdleControllerConfig(const DynamicIdleController::config_t& config)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);
    _preferences.putBytes("DIC", &config, sizeof(config));
    _preferences.end();
#else
    (void)config;
#endif
}

RadioController::rates_t NonVolatileStorage::loadRadioControllerRates() const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_ONLY);
    if (_preferences.isKey("RCR")) {
        RadioController::rates_t config {};
        _preferences.getBytes("RCR", &config, sizeof(config));
        _preferences.end();
        return config;
    }
    _preferences.end();
#endif
    return DEFAULTS::radioControllerRates;
}

void NonVolatileStorage::storeRadioControllerRates(const RadioController::rates_t& config)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);
    _preferences.putBytes("RCR", &config, sizeof(config));
    _preferences.end();
#else
    (void)config;
#endif
}

bool NonVolatileStorage::isSetPID() const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_ONLY);

    const bool ret = _preferences.getBool("PIDS_SET", false);

    _preferences.end();
#else
    const bool ret = false;
#endif
    return ret;
}

PIDF::PIDF_t NonVolatileStorage::getPID(const std::string& name) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_ONLY);

    const PIDF::PIDF_t pid {
        .kp = _preferences.getFloat((name + "_P").c_str(), NOT_SET),
        .ki = _preferences.getFloat((name + "_I").c_str(), NOT_SET),
        .kd = _preferences.getFloat((name + "_D").c_str(), NOT_SET),
        .kf = _preferences.getFloat((name + "_F").c_str(), NOT_SET),
        .ks = _preferences.getFloat((name + "_S").c_str(), NOT_SET)
    };

    _preferences.end();
#else
    (void)name;
    const PIDF::PIDF_t pid {};
#endif
    return pid;
}

void NonVolatileStorage::putPID(const std::string& name, const PIDF::PIDF_t& pid)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);

    _preferences.putBool("PIDS_SET", true);
    _preferences.putFloat((name + "_P").c_str(), pid.kp);
    _preferences.putFloat((name + "_I").c_str(), pid.ki);
    _preferences.putFloat((name + "_D").c_str(), pid.kd);
    _preferences.putFloat((name + "_F").c_str(), pid.kf);
    _preferences.putFloat((name + "_S").c_str(), pid.ks);

    _preferences.end();
#else
    (void)name;
    (void)pid;
#endif
}

float NonVolatileStorage::getFloat(const std::string& name) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_ONLY);

    const float pitchBalanceAngleDegrees = _preferences.getFloat(name.c_str(), NOT_SET);

    _preferences.end();
#else
    (void)name;
    const float pitchBalanceAngleDegrees =  0.0F;
#endif
    return pitchBalanceAngleDegrees;
}

void NonVolatileStorage::putFloat(const std::string& name, float value)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);

    _preferences.putFloat(name.c_str(), value);

    _preferences.end();
#else
    (void)name;
    (void)value;
#endif
}

void NonVolatileStorage::removeAccOffset()
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);
    _preferences.remove("acc");
    _preferences.remove("acc_x");
    _preferences.remove("acc_y");
    _preferences.remove("acc_z");
#endif
}

bool NonVolatileStorage::getAccOffset(int32_t& x, int32_t& y, int32_t& z) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_ONLY);

    const bool ret = _preferences.getBool("acc", false);
    if (ret) {
        x = _preferences.getInt("acc_x", 0);
        y = _preferences.getInt("acc_y", 0);
        z = _preferences.getInt("acc_z", 0);
    }

    _preferences.end();
#else
    (void)x;
    (void)y;
    (void)z;
    const bool ret = false;
#endif
    return ret;
}

void NonVolatileStorage::putAccOffset(int32_t x, int32_t y, int32_t z)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);

    _preferences.putBool("acc", true);
    _preferences.putInt("acc_x", x);
    _preferences.putInt("acc_y", y);
    _preferences.putInt("acc_z", z);

    _preferences.end();
#else
    (void)x;
    (void)y;
    (void)z;
#endif
}

void NonVolatileStorage::removeGyroOffset()
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);
    _preferences.remove("gyro");
    _preferences.remove("gyro_x");
    _preferences.remove("gyro_y");
    _preferences.remove("gyro_z");
#endif
}

bool NonVolatileStorage::getGyroOffset(int32_t& x, int32_t& y, int32_t& z) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_ONLY);

    const bool ret = _preferences.getBool("gyro", false);
    if (ret) {
        x = _preferences.getInt("gyro_x", 0);
        y = _preferences.getInt("gyro_y", 0);
        z = _preferences.getInt("gyro_z", 0);
    }

    _preferences.end();
#else
    (void)x;
    (void)y;
    (void)z;
    const bool ret = false;
#endif
    return ret;
}

void NonVolatileStorage::putGyroOffset(int32_t x, int32_t y, int32_t z)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);

    _preferences.putBool("gyro", true);
    _preferences.putInt("gyro_x", x);
    _preferences.putInt("gyro_y", y);
    _preferences.putInt("gyro_z", z);

    _preferences.end();
#else
    (void)x;
    (void)y;
    (void)z;
#endif
}

void NonVolatileStorage::getMacAddress(uint8_t* macAddress, const std::string& name) const // NOLINT(readability-non-const-parameter)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_ONLY);

    _preferences.getBytes(name.c_str(), macAddress, MAC_ADDRESS_LEN);

    _preferences.end();
#else
    (void)macAddress;
    (void)name;
#endif
}

void NonVolatileStorage::putMacAddress(const std::string& name, const uint8_t* macAddress)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(nonVolatileStorageNamespace, READ_WRITE);

    _preferences.putBytes(name.c_str(), macAddress, MAC_ADDRESS_LEN);

    _preferences.end();
#else
    (void)macAddress;
    (void)name;
#endif
}
