#include "Autopilot.h"
#include "Cockpit.h"
#include "Defaults.h"
#if defined(USE_CMS)
#include "CMS.h"
#endif
#if defined(USE_GPS)
#include "GPS.h"
#endif
#include "NonVolatileStorage.h"
#if defined(USE_OSD)
#include "OSD.h"
#endif
#include <ahrs.h>
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
constexpr uint16_t FLIGHT_CONTROLLER_ANTI_GRAVITY_CONFIG_KEY = 0x0410;
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

void NonVolatileStorage::to_hex_chars(char* charPtr, uint16_t value)
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
        to_hex_chars(&keyS[0], key);
        _preferences.remove(&keyS[0]);
        _preferences.end();
    }
    return OK;
#else
    (void)key;
    return OK;
#endif
}


bool NonVolatileStorage::load_item(uint16_t key, void* item, size_t length) const
{
#if defined(USE_FLASH_KLV)
    if (FlashKLV::OK == _flashKLV.read(item, length, key)) {
        return true;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        std::array<char, 8> keyS;
        to_hex_chars(&keyS[0], key);
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

bool NonVolatileStorage::load_item(uint16_t key, uint8_t pid_profile_index, void* item, size_t length) const
{
    if (pid_profile_index >= PID_PROFILE_COUNT) {
        return false;
    }
    return load_item(key + pid_profile_index, item, length);
}

int32_t NonVolatileStorage::store_item(uint16_t key, const void* item, size_t length, const void* defaults)
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
        to_hex_chars(&keyS[0], key);
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

int32_t NonVolatileStorage::store_item(uint16_t key, uint8_t pid_profile_index, const void* item, size_t length, const void* defaults)
{
    if (pid_profile_index >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    return store_item(key + pid_profile_index, item, length, defaults);
}


uint8_t NonVolatileStorage::load_pid_profile_index() const
{
    uint8_t profile_index {};
    if (load_item(PID_PROFILE_INDEX_KEY, &profile_index, sizeof(profile_index))) { // cppcheck-suppress knownConditionTrueFalse
        return profile_index;
    }
    return DEFAULT_PID_PROFILE;
}

int32_t NonVolatileStorage::store_pid_profile_index(uint8_t profile_index)
{
    if (profile_index >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint8_t default_profile_index = DEFAULT_PID_PROFILE;
    return store_item(PID_PROFILE_INDEX_KEY, &profile_index, sizeof(profile_index), &default_profile_index);
}


uint8_t NonVolatileStorage::load_rate_profile_index() const
{
    uint8_t profile_index {};
    if (load_item(RATE_PROFILE_INDEX_KEY, &profile_index, sizeof(profile_index))) { // cppcheck-suppress knownConditionTrueFalse
        return profile_index;
    }
    return DEFAULT_RATE_PROFILE;
}

int32_t NonVolatileStorage::store_rate_profile_index(uint8_t profile_index)
{
    if (profile_index >= RATE_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint8_t default_profile_index = DEFAULT_RATE_PROFILE;
    return store_item(RATE_PROFILE_INDEX_KEY, &profile_index, sizeof(profile_index), &default_profile_index);
}


dynamic_idle_controller_config_t NonVolatileStorage::load_dynamic_idle_controller_config(uint8_t pid_profile_index) const
{
    {dynamic_idle_controller_config_t config {};
    if (load_item(DYNAMIC_IDLE_CONTROLLER_CONFIG_KEY, pid_profile_index, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::DYNAMIC_IDLE_CONTROLLER_CONFIG;
}

int32_t NonVolatileStorage::store_dynamic_idle_controller_config(const dynamic_idle_controller_config_t& config, uint8_t pid_profile_index)
{
    return store_item(DYNAMIC_IDLE_CONTROLLER_CONFIG_KEY, pid_profile_index, &config, sizeof(config), &DEFAULTS::DYNAMIC_IDLE_CONTROLLER_CONFIG);
}

mixer_config_t NonVolatileStorage::load_motor_mixer_config() const
{
    {mixer_config_t config {};
    if (load_item(DYNAMIC_IDLE_CONTROLLER_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::MOTOR_MIXER_CONFIG;
}

int32_t NonVolatileStorage::store_motor_mixer_config(const mixer_config_t& config)
{
    return store_item(MOTOR_MIXER_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::MOTOR_MIXER_CONFIG);
}


motor_config_t NonVolatileStorage::load_motor_config() const
{
    {motor_config_t config {};
    if (load_item(MOTOR_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::MOTOR_CONFIG;
}

int32_t NonVolatileStorage::store_motor_config(const motor_config_t& config)
{
    return store_item(MOTOR_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::MOTOR_CONFIG);
}


flight_controller_filters_config_t NonVolatileStorage::load_flight_controller_filters_config(uint8_t pid_profile_index) const
{
    {flight_controller_filters_config_t config {};
    if (load_item(FLIGHT_CONTROLLER_FILTERS_CONFIG_KEY, pid_profile_index, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_FILTERS_CONFIG;
}

int32_t NonVolatileStorage::store_flight_controller_filters_config(const flight_controller_filters_config_t& config, uint8_t pid_profile_index)
{
    return store_item(FLIGHT_CONTROLLER_FILTERS_CONFIG_KEY, pid_profile_index, &config, sizeof(config), &DEFAULTS::FLIGHT_CONTROLLER_FILTERS_CONFIG);
}


flight_mode_config_t NonVolatileStorage::load_flight_controller_flight_mode_config(uint8_t pid_profile_index) const
{
    {flight_mode_config_t config {};
    if (load_item(FLIGHT_CONTROLLER_FLIGHTMODE_CONFIG_KEY, pid_profile_index, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_FLIGHT_MODE_CONFIG;
}

int32_t NonVolatileStorage::store_flight_controller_flight_mode_config(const flight_mode_config_t& config, uint8_t pid_profile_index)
{
    return store_item(FLIGHT_CONTROLLER_FLIGHTMODE_CONFIG_KEY, pid_profile_index, &config, sizeof(config), &DEFAULTS::FLIGHT_CONTROLLER_FLIGHT_MODE_CONFIG);
}


tpa_config_t NonVolatileStorage::load_flight_controller_tpa_config(uint8_t pid_profile_index) const
{
    {tpa_config_t config {};
    if (load_item(FLIGHT_CONTROLLER_TPA_CONFIG_KEY, pid_profile_index, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_TPA__CONFIG;
}

int32_t NonVolatileStorage::store_flight_controller_tpa_config(const tpa_config_t& config, uint8_t pid_profile_index)
{
    return store_item(FLIGHT_CONTROLLER_TPA_CONFIG_KEY, pid_profile_index, &config, sizeof(config), &DEFAULTS::FLIGHT_CONTROLLER_TPA__CONFIG);
}


anti_gravity_config_t NonVolatileStorage::load_flight_controller_anti_gravity_config(uint8_t pid_profile_index) const
{
    {anti_gravity_config_t config {};
    if (load_item(FLIGHT_CONTROLLER_ANTI_GRAVITY_CONFIG_KEY, pid_profile_index, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_ANTI_GRAVITY_CONFIG;
}

int32_t NonVolatileStorage::store_flight_controller_anti_gravity_config(const anti_gravity_config_t& config, uint8_t pid_profile_index)
{
    return store_item(FLIGHT_CONTROLLER_ANTI_GRAVITY_CONFIG_KEY, pid_profile_index, &config, sizeof(config), &DEFAULTS::FLIGHT_CONTROLLER_ANTI_GRAVITY_CONFIG);
}


crash_flip_config_t NonVolatileStorage::load_flight_controller_crash_flip_config() const
{
    {crash_flip_config_t config {};
    if (load_item(FLIGHT_CONTROLLER_CRASH_FLIP_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_CRASH_FLIP_CONFIG;
}

int32_t NonVolatileStorage::store_flight_controller_crash_flip_config(const crash_flip_config_t& config)
{
    return store_item(FLIGHT_CONTROLLER_CRASH_FLIP_KEY, &config, sizeof(config), &DEFAULTS::FLIGHT_CONTROLLER_CRASH_FLIP_CONFIG);
}


#if defined(USE_D_MAX)
d_max_config_t NonVolatileStorage::load_flight_controller_d_max_config(uint8_t pid_profile_index) const
{
    {d_max_config_t config {};
    if (load_item(FLIGHT_CONTROLLER_DMAX_CONFIG_KEY, pid_profile_index, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_D_MAX_CONFIG;
}

int32_t NonVolatileStorage::store_flight_controller_d_max_config(const d_max_config_t& config, uint8_t pid_profile_index)
{
    return store_item(FLIGHT_CONTROLLER_DMAX_CONFIG_KEY, pid_profile_index, &config, sizeof(config), &DEFAULTS::FLIGHT_CONTROLLER_D_MAX_CONFIG);
}
#endif


#if defined(USE_ITERM_RELAX)
iterm_relax_config_t NonVolatileStorage::load_flight_controller_iterm_relax_config(uint8_t pid_profile_index) const
{
    {iterm_relax_config_t config {};
    if (load_item(FLIGHT_CONTROLLER_ITERM_RELAX_CONFIG_KEY, pid_profile_index, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_ITERM_RELAX_CONFIG;
}

int32_t NonVolatileStorage::store_flight_controller_iterm_relax_config(const iterm_relax_config_t& config, uint8_t pid_profile_index)
{
    return store_item(FLIGHT_CONTROLLER_ITERM_RELAX_CONFIG_KEY, pid_profile_index, &config, sizeof(config), &DEFAULTS::FLIGHT_CONTROLLER_ITERM_RELAX_CONFIG);
}
#endif


#if defined(USE_YAW_SPIN_RECOVERY)
yaw_spin_recovery_config_t NonVolatileStorage::load_flight_controller_yaw_spin_recovery_config(uint8_t pid_profile_index) const
{
    {yaw_spin_recovery_config_t config {};
    if (load_item(FLIGHT_CONTROLLER_YAW_SPIN_RECOVERY_CONFIG_KEY, pid_profile_index, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_YAW_SPIN_RECOVERY_CONFIG;
}

int32_t NonVolatileStorage::store_flight_controller_yaw_spin_recovery_config(const yaw_spin_recovery_config_t& config, uint8_t pid_profile_index)
{
    return store_item(FLIGHT_CONTROLLER_YAW_SPIN_RECOVERY_CONFIG_KEY, pid_profile_index, &config, sizeof(config), &DEFAULTS::FLIGHT_CONTROLLER_YAW_SPIN_RECOVERY_CONFIG);
}
#endif


#if defined(USE_CRASH_RECOVERY)
crash_recovery_config_t NonVolatileStorage::load_flight_controller_crash_recovery_config(uint8_t pid_profile_index) const
{
    {crash_recovery_config_t config {};
    if (load_item(FLIGHT_CONTROLLER_CRASH_RECOVERY_CONFIG_KEY, pid_profile_index, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_CRASH_RECOVERY_CONFIG;
}

int32_t NonVolatileStorage::store_flight_controller_crash_recovery_config(const crash_recovery_config_t& config, uint8_t pid_profile_index)
{
    return store_item(FLIGHT_CONTROLLER_CRASH_RECOVERY_CONFIG_KEY, pid_profile_index, &config, sizeof(config), &DEFAULTS::FLIGHT_CONTROLLER_CRASH_RECOVERY_CONFIG);
}
#endif


#if defined(USE_DYNAMIC_NOTCH_FILTER)
dynamic_notch_filter_config_t NonVolatileStorage::load_dynamic_notch_filter_config() const
{
    {dynamic_notch_filter_config_t config {};
    if (load_item(DYNAMIC_NOTCH_FILTER_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::DYNAMIC_NOTCH_FILTER_CONFIG;
}

int32_t NonVolatileStorage::store_dynamic_notch_filter_config(const dynamic_notch_filter_config_t& config)
{
    return store_item(DYNAMIC_NOTCH_FILTER_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::DYNAMIC_NOTCH_FILTER_CONFIG);
}
#endif

#if defined(USE_RPM_FILTERS)
rpm_filters_config_t NonVolatileStorage::load_rpm_filters_config() const
{
    {rpm_filters_config_t config {};
    if (load_item(RPM_FILTERS_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::RPM_FILTERS_CONFIG;
}

int32_t NonVolatileStorage::store_rpm_filters_config(const rpm_filters_config_t& config)
{
    return store_item(RPM_FILTERS_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::RPM_FILTERS_CONFIG);
}
#endif
#if defined(USE_OSD)
osd_config_t NonVolatileStorage::load_osd_config() const
{
    {osd_config_t config {};
    if (load_item(OSD_ELEMENTS_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::OSD_CONFIG;
}

int32_t NonVolatileStorage::store_osd_config(const osd_config_t& config)
{
    return store_item(OSD_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::OSD_CONFIG);
}

bool NonVolatileStorage::load_osd_elements_config(osd_elements_config_t& config) const
{
    if (load_item(OSD_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return true;
    }
    return false;
}

int32_t NonVolatileStorage::store_osd_elements_config(const osd_elements_config_t& config)
{
    return store_item(OSD_ELEMENTS_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::OSD_ELEMENTS_CONFIG);
}
#endif
#if defined(USE_VTX)
vtx_config_t NonVolatileStorage::load_vtx_config() const
{
    {vtx_config_t config {};
    if (load_item(VTX_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::VTX_CONFIG;
}
int32_t NonVolatileStorage::store_vtx_config(const vtx_config_t& config)
{
    return store_item(VTX_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::VTX_CONFIG);
}
#endif
#if defined(USE_GPS)
gps_config_t NonVolatileStorage::load_gps_config() const
{
    {gps_config_t config {};
    if (load_item(GPS_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::GPS_CONFIG;
}
int32_t NonVolatileStorage::store_gps_config(const gps_config_t& config)
{
    return store_item(GPS_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::GPS_CONFIG);
}
#endif
#if defined(USE_ALTITUDE_HOLD)
autopilot_config_t NonVolatileStorage::load_autopilot_config() const
{
    {autopilot_config_t config {};
    if (load_item(AUTOPILOT_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::AUTOPILOT_CONFIG;
}

int32_t NonVolatileStorage::store_autopilot_config(const autopilot_config_t& config)
{
    return store_item(AUTOPILOT_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::AUTOPILOT_CONFIG);
}

position_hold_config_t NonVolatileStorage::load_autopilot_position_hold_config() const
{
    {position_hold_config_t config {};
    if (load_item(AUTOPILOT_POSITION_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::AUTOPILOT_POSITION_HOLD_CONFIG;
}

int32_t NonVolatileStorage::store_autopilot_position_hold_config(const position_hold_config_t& config)
{
    return store_item(AUTOPILOT_POSITION_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::AUTOPILOT_POSITION_HOLD_CONFIG);
}

altitude_hold_config_t NonVolatileStorage::load_altitude_hold_config() const
{
    {altitude_hold_config_t config {};
    if (load_item(ALTITUDE_HOLD_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::AUTOPILOT_ALTITUDE_HOLD_CONFIG;
}

int32_t NonVolatileStorage::store_altitude_hold_config(const altitude_hold_config_t& config)
{
    return store_item(ALTITUDE_HOLD_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::AUTOPILOT_ALTITUDE_HOLD_CONFIG);
}
#endif


imu_filters_config_t NonVolatileStorage::load_imu_filters_config() const
{
    {imu_filters_config_t config {};
    if (load_item(IMU_FILTERS_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::IMU_FILTERS_CONFIG;
}

int32_t NonVolatileStorage::store_imu_filters_config(const imu_filters_config_t& config)
{
    return store_item(IMU_FILTERS_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::IMU_FILTERS_CONFIG);
}


failsafe_config_t NonVolatileStorage::load_failsafe_config() const
{
    {failsafe_config_t config {};
    if (load_item(FAILSAFE_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::COCKPIT_FAILSAFE_CONFIG;
}

int32_t NonVolatileStorage::store_failsafe_config(const failsafe_config_t& config)
{
    return store_item(FAILSAFE_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::COCKPIT_FAILSAFE_CONFIG);
}

features_config_t NonVolatileStorage::load_features_config() const
{
    {features_config_t config {};
    if (load_item(FEATURES_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::FEATURES_CONFIG;
}

int32_t NonVolatileStorage::store_features_config(const features_config_t& config)
{
    return store_item(FEATURES_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::FEATURES_CONFIG);
}

rx_config_t NonVolatileStorage::load_rx_config() const
{
    {rx_config_t config {};
    if (load_item(RX_CONFIG_KEY, &config, sizeof(config))) { // cppcheck-suppress knownConditionTrueFalse
        return config;
    }}
    return DEFAULTS::RX_CONFIG;
}

int32_t NonVolatileStorage::store_rx_config(const rx_config_t& config)
{
    return store_item(RX_CONFIG_KEY, &config, sizeof(config), &DEFAULTS::RX_CONFIG);
}


rc_modes_activation_condition_array_t NonVolatileStorage::load_rc_mode_activation_conditions() const
{
    {rc_modes_activation_condition_array_t mode_activation_conditions {};
    if (load_item(RC_MODES_ACTIVATION_CONDITIONS_KEY, &mode_activation_conditions, sizeof(mode_activation_conditions))) {
        return mode_activation_conditions;
    }}
    return DEFAULTS::RC_MODE_ACTIVATION_CONDITIONS;
}

int32_t NonVolatileStorage::store_rc_mode_activation_conditions(const rc_modes_activation_condition_array_t& mode_activation_conditions)
{
    return store_item(RC_MODES_ACTIVATION_CONDITIONS_KEY, &mode_activation_conditions, sizeof(mode_activation_conditions), &DEFAULTS::RC_MODE_ACTIVATION_CONDITIONS);
}

#if defined(USE_RC_ADJUSTMENTS)
rc_adjustment_ranges_t NonVolatileStorage::load_rc_adjustment_ranges() const
{
    {rc_adjustment_ranges_t adjustmentRanges {};
    if (load_item(RC_ADJUSTMENT_RANGES_KEY, &adjustmentRanges, sizeof(adjustmentRanges))) {
        return adjustmentRanges;
    }}
    return DEFAULTS::RC_ADJUSTMENT_RANGES;
}

int32_t NonVolatileStorage::store_rc_adjustment_ranges(const rc_adjustment_ranges_t& adjustmentRanges)
{
    return store_item(RC_ADJUSTMENT_RANGES_KEY, &adjustmentRanges, sizeof(adjustmentRanges), &DEFAULTS::RC_ADJUSTMENT_RANGES);
}
#endif

rates_t NonVolatileStorage::load_rates(uint8_t rate_profile_index) const
{
    {rates_t rates {};
    if (rate_profile_index < RATE_PROFILE_COUNT && load_item(RATES_KEY + rate_profile_index, &rates, sizeof(rates))) { // cppcheck-suppress knownConditionTrueFalse
        return rates;
    }}
    return DEFAULTS::COCKPIT_RATES;
}

int32_t NonVolatileStorage::store_rates(const rates_t& rates, uint8_t rate_profile_index)
{
    if (rate_profile_index >= RATE_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint16_t key = RATES_KEY + rate_profile_index;
    return store_item(key, &rates, sizeof(rates), &DEFAULTS::COCKPIT_RATES);
}

VehicleControllerBase::PIDF_uint16_t NonVolatileStorage::load_pid(uint8_t pid_index, uint8_t pid_profile_index) const
{
    assert(pid_index <= FlightController::PID_COUNT);
    {VehicleControllerBase::PIDF_uint16_t pid {};
    if (pid_profile_index < PID_PROFILE_COUNT && load_item(PID_Keys[pid_index] + pid_profile_index, &pid, sizeof(pid))) { // cppcheck-suppress knownConditionTrueFalse
        return pid;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_PIDS[pid_index];
}

int32_t NonVolatileStorage::store_pid(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pid_index, uint8_t pid_profile_index)
{
    assert(pid_index <= FlightController::PID_COUNT);
    if (pid_profile_index >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint16_t key = PID_Keys[pid_index] + pid_profile_index;
    return store_item(key, &pid, sizeof(pid), &DEFAULTS::FLIGHT_CONTROLLER_PIDS[pid_index]);
}

void NonVolatileStorage::reset_pid(uint8_t pid_index, uint8_t pid_profile_index)
{
    assert(pid_index <= FlightController::PID_COUNT);
    assert(pid_profile_index < PID_PROFILE_COUNT);
    remove(PID_Keys[pid_index]);
    (void)pid_profile_index; //!!TODO: check if this is needed
}

simplified_pid_settings_t NonVolatileStorage::load_simplified_pid_settings(uint8_t pid_profile_index) const
{
    assert(pid_profile_index < PID_PROFILE_COUNT);
    {simplified_pid_settings_t settings {};
    if (pid_profile_index < PID_PROFILE_COUNT && load_item(FLIGHT_CONTROLLER_SIMPLIFIED_PID_settingsKey + pid_profile_index, &settings, sizeof(settings))) { // cppcheck-suppress knownConditionTrueFalse
        return settings;
    }}
    return DEFAULTS::FLIGHT_CONTROLLER_SIMPLIFIED_PID_SETTINGS;
}

int32_t NonVolatileStorage::store_simplified_pid_settings(const simplified_pid_settings_t& settings, uint8_t pid_profile_index)
{
    assert(pid_profile_index < PID_PROFILE_COUNT);
    const uint16_t key = FLIGHT_CONTROLLER_SIMPLIFIED_PID_settingsKey + pid_profile_index;
    return store_item(key, &settings, sizeof(settings), &DEFAULTS::FLIGHT_CONTROLLER_SIMPLIFIED_PID_SETTINGS);
}


NonVolatileStorage::calibration_state_e NonVolatileStorage::load_acc_calibration_state() const
{
    calibration_state_e calibration_state {};
    if (load_item(ACC_CALIBRATION_STATE_KEY, &calibration_state, sizeof(calibration_state))) { // cppcheck-suppress knownConditionTrueFalse
        return calibration_state;
    }
    return NOT_CALIBRATED;
}

int32_t NonVolatileStorage::store_acc_calibration_state(calibration_state_e calibration_state)
{
    const calibration_state_e defaultCalibration_state = NOT_CALIBRATED;
    return store_item(ACC_CALIBRATION_STATE_KEY, &calibration_state, sizeof(calibration_state), &defaultCalibration_state);
}

xyz_t NonVolatileStorage::load_acc_offset() const
{
    {xyz_t offset {};
    if (load_item(ACC_OFFSET_KEY, &offset, sizeof(offset))) { // cppcheck-suppress knownConditionTrueFalse
        return offset;
    }}
    return xyz_t { .x = 0.0F, .y = 0.0F, .z = 0.0F };
}

int32_t NonVolatileStorage::store_acc_offset(const xyz_t& offset)
{
    const xyz_t defaultOffset = { .x = 0.0F, .y = 0.0F, .z = 0.0F };
    return store_item(ACC_OFFSET_KEY, &offset, sizeof(offset), &defaultOffset);
}

NonVolatileStorage::calibration_state_e NonVolatileStorage::load_gyro_calibration_state() const
{
    calibration_state_e calibration_state {};
    if (load_item(GYRO_CALIBRATION_STATE_KEY, &calibration_state, sizeof(calibration_state))) { // cppcheck-suppress knownConditionTrueFalse
        return calibration_state;
    }
    return NOT_CALIBRATED;
}

int32_t NonVolatileStorage::store_gyro_calibration_state(calibration_state_e calibration_state)
{
    const calibration_state_e defaultCalibration_state = NOT_CALIBRATED;
    return store_item(GYRO_CALIBRATION_STATE_KEY, &calibration_state, sizeof(calibration_state), &defaultCalibration_state);
}

xyz_t NonVolatileStorage::load_gyro_offset() const
{
    {xyz_t offset {};
    if (load_item(GYRO_OFFSET_KEY, &offset, sizeof(offset))) { // cppcheck-suppress knownConditionTrueFalse
        return offset;
    }}
    return xyz_t { .x = 0.0F, .y = 0.0F, .z = 0.0F };
}

int32_t NonVolatileStorage::store_gyro_offset(const xyz_t& offset)
{
    const xyz_t defaultOffset = { .x = 0.0F, .y = 0.0F, .z = 0.0F };
    return store_item(GYRO_OFFSET_KEY, &offset, sizeof(offset), &defaultOffset);
}

void NonVolatileStorage::load_mac_address(uint8_t* mac_address) const // NOLINT(readability-non-const-parameter)
{
#if defined(USE_FLASH_KLV)
    (void)mac_address;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        std::array<char, 8> keyS;
        to_hex_chars(&keyS[0], MAC_ADDRESS_KEY);
        _preferences.getBytes(&keyS[0], mac_address, MAC_ADDRESS_LEN);
        _preferences.end();
    }
#else
    (void)mac_address;
#endif
}

int32_t NonVolatileStorage::store_mac_address(const uint8_t* mac_address)
{
#if defined(USE_FLASH_KLV)
    (void)mac_address;
    return OK;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        std::array<char, 8> keyS;
        to_hex_chars(&keyS[0], MAC_ADDRESS_KEY);
        _preferences.putBytes(&keyS[0], mac_address, MAC_ADDRESS_LEN);
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)mac_address;
    return OK;
#endif
}

int32_t NonVolatileStorage::store_all(const IMU_Filters& imuFilters, const FlightController& flightController, const MotorMixerBase& motorMixer, const Cockpit& cockpit, const RcModes& rc_modes)
{
    (void)motorMixer;
#if defined(USE_DYNAMIC_IDLE)
    const dynamic_idle_controller_config_t* dynamicIdleControllerConfig = motorMixer.get_dynamic_idle_config();
    if (dynamicIdleControllerConfig) {
        store_dynamic_idle_controller_config(*dynamicIdleControllerConfig, _current_pid_profile_index);
    }
#endif
#if defined(USE_RPM_FILTERS)
    const RpmFilters* rpmFilters = imuFilters.getRPM_Filters();
    if (rpmFilters) {
        store_rpm_filters_config(rpmFilters->get_config());
    }
#endif

    store_flight_controller_filters_config(flightController.getFiltersConfig(), _current_pid_profile_index);

    store_flight_controller_flight_mode_config(flightController.getFlightModeConfig(), _current_pid_profile_index);

    store_flight_controller_tpa_config(flightController.getTPA_Config(), _current_pid_profile_index);

    store_flight_controller_anti_gravity_config(flightController.getAntiGravityConfig(), _current_pid_profile_index);

    store_flight_controller_crash_flip_config(flightController.getCrashFlipConfig());

#if defined(USE_D_MAX)
    store_flight_controller_d_max_config(flightController.getDMaxConfig(), _current_pid_profile_index);
#endif
#if defined(USE_ITERM_RELAX)
    store_flight_controller_iterm_relax_config(flightController.getITermRelaxConfig(), _current_pid_profile_index);
#endif
#if defined(USE_YAW_SPIN_RECOVERY)
    store_flight_controller_yaw_spin_recovery_config(flightController.getYawSpinRecoveryConfig(), _current_pid_profile_index);
#endif
#if defined(USE_CRASH_RECOVERY)
    store_flight_controller_crash_recovery_config(flightController.getCrashRecoveryConfig(), _current_pid_profile_index);
#endif
#if defined(USE_ALTITUDE_HOLD)
    store_altitude_hold_config(cockpit.getAutopilot().get_altitude_hold_config());
#endif

    const imu_filters_config_t imu_filters_config = imuFilters.getConfig();
    store_imu_filters_config(imu_filters_config);

#if defined(USE_DYNAMIC_NOTCH_FILTER)
    store_dynamic_notch_filter_config(imuFilters.get_dynamic_notch_filter_config());
#endif

    store_rates(cockpit.getRates(), _current_rate_profile_index);
    store_rc_mode_activation_conditions(rc_modes.get_mode_activation_conditions());
#if defined(USE_RC_ADJUSTMENTS)
    store_rc_adjustment_ranges(cockpit.getRC_Adjustments().getAdjustmentRanges());
#endif
    store_features_config(cockpit.get_features_config());

    return OK;
}

#if defined(USE_CMS)
void CMSX::saveConfigAndNotify(cms_parameter_group_t& pg)
{
    pg.nvs.store_all(pg.imuFilters, pg.flightController, pg.motorMixer, pg.cockpit, pg.rc_modes);
}

uint8_t CMSX::get_current_pid_profile_index(cms_parameter_group_t& pg) const
{
    return pg.nvs.get_current_pid_profile_index();
}

void CMSX::set_current_pid_profile_index(cms_parameter_group_t& pg, uint8_t current_pid_profile_index) 
{
    pg.nvs.set_current_pid_profile_index(current_pid_profile_index);
}

uint8_t CMSX::get_current_rate_profile_index(cms_parameter_group_t& pg) const
{
    return pg.nvs.get_current_rate_profile_index();
}

void CMSX::set_current_rate_profile_index(cms_parameter_group_t& pg, uint8_t current_rate_profile_index) 
{
    pg.nvs.set_current_rate_profile_index(current_rate_profile_index);
}
#endif
