#include "SV_Preferences.h"

#if defined(USE_ARDUINO_ESP32_PREFERENCES)
namespace { // use anonymous namespace to make items local to this translation unit
const char* preferencesNamespace {"SV"};
} // end namespace
#endif


void SV_Preferences::clear()
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.clear();

    _preferences.end();
#endif
}

bool SV_Preferences::isSetPID() const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const bool ret = _preferences.getBool("PIDS_SET", false);

    _preferences.end();
#else
    const bool ret = false;
#endif
    return ret;
}

PIDF::PIDF_t SV_Preferences::getPID(const std::string& name) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_ONLY);

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

void SV_Preferences::putPID(const std::string& name, const PIDF::PIDF_t& pid)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_WRITE);

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

float SV_Preferences::getFloat(const std::string& name) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const float pitchBalanceAngleDegrees = _preferences.getFloat(name.c_str(), NOT_SET);

    _preferences.end();
#else
    (void)name;
    const float pitchBalanceAngleDegrees =  0.0F;
#endif
    return pitchBalanceAngleDegrees;
}

void SV_Preferences::putFloat(const std::string& name, float value)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putFloat(name.c_str(), value);

    _preferences.end();
#else
    (void)name;
    (void)value;
#endif
}

void SV_Preferences::removeAccOffset()
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_WRITE);
    _preferences.remove("acc");
    _preferences.remove("acc_x");
    _preferences.remove("acc_y");
    _preferences.remove("acc_z");
#endif
}

bool SV_Preferences::getAccOffset(int32_t& x, int32_t& y, int32_t& z) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_ONLY);

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

void SV_Preferences::putAccOffset(int32_t x, int32_t y, int32_t z)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_WRITE);

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

void SV_Preferences::removeGyroOffset()
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_WRITE);
    _preferences.remove("gyro");
    _preferences.remove("gyro_x");
    _preferences.remove("gyro_y");
    _preferences.remove("gyro_z");
#endif
}

bool SV_Preferences::getGyroOffset(int32_t& x, int32_t& y, int32_t& z) const
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_ONLY);

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

void SV_Preferences::putGyroOffset(int32_t x, int32_t y, int32_t z)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_WRITE);

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

void SV_Preferences::getMacAddress(uint8_t* macAddress, const std::string& name) const // NOLINT(readability-non-const-parameter)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_ONLY);

    _preferences.getBytes(name.c_str(), macAddress, MAC_ADDRESS_LEN);

    _preferences.end();
#else
    (void)macAddress;
    (void)name;
#endif
}

void SV_Preferences::putMacAddress(const std::string& name, const uint8_t* macAddress)
{
#if defined(USE_ARDUINO_ESP32_PREFERENCES)
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBytes(name.c_str(), macAddress, MAC_ADDRESS_LEN);

    _preferences.end();
#else
    (void)macAddress;
    (void)name;
#endif
}
