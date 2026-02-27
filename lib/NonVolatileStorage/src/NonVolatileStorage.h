#pragma once


#include "RC_Adjustments.h"
#include "RC_Modes.h"
//#include <motor_mixer_base.h>

#if defined(USE_FLASH_KLV)

#include <FlashKlv.h>

#else

// ESP32 and/or ARDUINO_ARCH_ESP32 are defined by Arduino in platform.txt
//#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
//#if !defined(USE_ARDUINO_ESP32_PREFERENCES)
//#define USE_ARDUINO_ESP32_PREFERENCES
//#endif
//#endif

#if defined(USE_ARDUINO_ESP32_PREFERENCES)
#include <Preferences.h>
#endif

#endif // USE_FLASH_KLV

#include <cstdint>
#include <string>
#include <vehicle_controller_base.h>
#include <xyz_type.h>


class Cockpit;
class IMU_Filters;

struct altitude_hold_config_t;
struct autopilot_config_t;
struct dynamic_notch_filter_config_t;
struct failsafe_config_t;
struct features_config_t;
struct gps_config_t;
struct gps_rescue_config_t;
struct imu_filters_config_t;
struct osd_config_t;
struct osd_elements_config_t;
struct position_hold_config_t;
struct rates_t;
struct rx_config_t;

struct flight_controller_filters_config_t;
struct flight_mode_config_t;
struct tpa_config_t;
struct anti_gravity_config_t;
struct crash_flip_config_t;
struct yaw_spin_recovery_config_t;
struct crash_recovery_config_t;
struct iterm_relax_config_t;
struct d_max_config_t;
struct simplified_pid_settings_t;

struct vtx_config_t;

struct dynamic_idle_controller_config_t;
struct mixer_config_t;
struct motor_config_t;
struct rpm_filters_config_t;

/*!
*/
class NonVolatileStorage {
public:
    enum {
        OK = 0, OK_IS_DEFAULT,
        ERROR_FLASH_FULL = -1, ERROR_NOT_FOUND = -2, ERROR_NOT_WRITTEN = -3, ERROR_INVALID_PROFILE
    };

    enum { READ_WRITE=false, READ_ONLY=true };
    enum { MAC_ADDRESS_LEN = 6 };

    enum calibration_state_e { NOT_CALIBRATED = 0, CALIBRATED = 1 };
    enum { DEFAULT_PID_PROFILE = 0, PID_PROFILE_COUNT = 4 };
    enum { DEFAULT_RATE_PROFILE = 0, RATE_PROFILE_COUNT = 4 };

public:
    // NOTE: "get" functions are declared const, since they are logically const, although not physically const

    NonVolatileStorage();
    explicit NonVolatileStorage(uint32_t flashMemorySize);
    void init();
    static void to_hex_chars(char* charPtr, uint16_t value);

    int32_t clear();
    int32_t remove(uint16_t key);

    bool load_item(uint16_t key, void* item, size_t length) const;
    bool load_item(uint16_t key, uint8_t pid_profile_index, void* item, size_t length) const;

    int32_t store_item(uint16_t key, const void* item, size_t length, const void* defaults);
    int32_t store_item(uint16_t key, uint8_t pid_profile_index, const void* item, size_t length, const void* defaults);

    int32_t store_all(const IMU_Filters& imuFilters, const FlightController& flightController, const MotorMixerBase& motorMixer, const Cockpit& cockpit, const RcModes& rc_modes);

    void load_mac_address(uint8_t* mac_address) const;
    int32_t store_mac_address(const uint8_t* mac_address);

    uint8_t get_current_pid_profile_index() const { return _current_pid_profile_index; }
    void set_current_pid_profile_index(uint8_t current_pid_profile_index) { _current_pid_profile_index = current_pid_profile_index; }
    uint8_t get_current_rate_profile_index() const { return _current_rate_profile_index; }
    void set_current_rate_profile_index(uint8_t current_rate_profile_index) { _current_rate_profile_index = current_rate_profile_index; }

    uint8_t load_pid_profile_index() const;
    int32_t store_pid_profile_index(uint8_t pid_profile_index);

    uint8_t load_rate_profile_index() const;
    int32_t store_rate_profile_index(uint8_t rate_profile_index);

    calibration_state_e load_acc_calibration_state() const;
    int32_t store_acc_calibration_state(calibration_state_e calibration_state);

    xyz_t load_acc_offset() const;
    int32_t store_acc_offset(const xyz_t& offset);

    calibration_state_e load_gyro_calibration_state() const;
    int32_t store_gyro_calibration_state(calibration_state_e calibration_state);

    xyz_t load_gyro_offset() const;
    int32_t store_gyro_offset(const xyz_t& offset);

    dynamic_idle_controller_config_t load_dynamic_idle_controller_config(uint8_t pid_profile_index) const;
    int32_t store_dynamic_idle_controller_config(const dynamic_idle_controller_config_t& config, uint8_t pid_profile_index);

    mixer_config_t load_motor_mixer_config() const;
    int32_t store_motor_mixer_config(const mixer_config_t& config);

    motor_config_t load_motor_config() const;
    int32_t store_motor_config(const motor_config_t& config);

    imu_filters_config_t load_imu_filters_config() const;
    int32_t store_imu_filters_config(const imu_filters_config_t& config);

    failsafe_config_t load_failsafe_config() const;
    int32_t store_failsafe_config(const failsafe_config_t& config);

    features_config_t load_features_config() const;
    int32_t store_features_config(const features_config_t& config);

    rx_config_t load_rx_config() const;
    int32_t store_rx_config(const rx_config_t& config);

    rc_modes_activation_condition_array_t load_rc_mode_activation_conditions() const;
    int32_t store_rc_mode_activation_conditions(const rc_modes_activation_condition_array_t& mode_activation_conditions);

#if defined(USE_RC_ADJUSTMENTS)
    rc_adjustment_ranges_t load_rc_adjustment_ranges() const;
    int32_t store_rc_adjustment_ranges(const rc_adjustment_ranges_t& adjustmentRanges);
#endif
    rates_t load_rates(uint8_t rate_profile_index) const;
    int32_t store_rates(const rates_t& rates, uint8_t rate_profile_index);

    VehicleControllerBase::PIDF_uint16_t load_pid(uint8_t pid_index, uint8_t pid_profile_index) const;
    int32_t store_pid(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pid_index, uint8_t pid_profile_index);
    int32_t store_pid(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pid_index) { return store_pid(pid, pid_index, _current_pid_profile_index); }
    void reset_pid(uint8_t pid_index, uint8_t pid_profile_index);
    void reset_pid(uint8_t pid_index) { reset_pid(pid_index, _current_pid_profile_index); }

    simplified_pid_settings_t load_simplified_pid_settings(uint8_t pid_profile_index) const;
    int32_t store_simplified_pid_settings(const simplified_pid_settings_t& settings, uint8_t pid_profile_index);

    flight_controller_filters_config_t load_flight_controller_filters_config(uint8_t pid_profile_index) const;
    int32_t store_flight_controller_filters_config(const flight_controller_filters_config_t& config, uint8_t pid_profile_index);

    flight_mode_config_t load_flight_controller_flight_mode_config(uint8_t pid_profile_index) const;
    int32_t store_flight_controller_flight_mode_config(const flight_mode_config_t& config, uint8_t pid_profile_index);

    tpa_config_t load_flight_controller_tpa_config(uint8_t pid_profile_index) const;
    int32_t store_flight_controller_tpa_config(const tpa_config_t& config, uint8_t pid_profile_index);

    anti_gravity_config_t load_flight_controller_anti_gravity_config(uint8_t pid_profile_index) const;
    int32_t store_flight_controller_anti_gravity_config(const anti_gravity_config_t& config, uint8_t pid_profile_index);

    crash_flip_config_t load_flight_controller_crash_flip_config() const;
    int32_t store_flight_controller_crash_flip_config(const crash_flip_config_t& config);

#if defined(USE_D_MAX)
    d_max_config_t load_flight_controller_d_max_config(uint8_t pid_profile_index) const;
    int32_t store_flight_controller_d_max_config(const d_max_config_t& config, uint8_t pid_profile_index);
#endif
#if defined(USE_ITERM_RELAX)
    iterm_relax_config_t load_flight_controller_iterm_relax_config(uint8_t pid_profile_index) const;
    int32_t store_flight_controller_iterm_relax_config(const iterm_relax_config_t& config, uint8_t pid_profile_index);
#endif
#if defined(USE_YAW_SPIN_RECOVERY)
    yaw_spin_recovery_config_t load_flight_controller_yaw_spin_recovery_config(uint8_t pid_profile_index) const;
    int32_t store_flight_controller_yaw_spin_recovery_config(const yaw_spin_recovery_config_t& config, uint8_t pid_profile_index);
#endif
#if defined(USE_CRASH_RECOVERY)
    crash_recovery_config_t load_flight_controller_crash_recovery_config(uint8_t pid_profile_index) const;
    int32_t store_flight_controller_crash_recovery_config(const crash_recovery_config_t& config, uint8_t pid_profile_index);
#endif
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    dynamic_notch_filter_config_t load_dynamic_notch_filter_config() const;
    int32_t store_dynamic_notch_filter_config(const dynamic_notch_filter_config_t& config);
#endif
#if defined(USE_RPM_FILTERS)
    rpm_filters_config_t load_rpm_filters_config() const;
    int32_t store_rpm_filters_config(const rpm_filters_config_t& config);
#endif
#if defined(USE_VTX)
    vtx_config_t load_vtx_config() const;
    int32_t store_vtx_config(const vtx_config_t& config);
#endif
#if defined(USE_OSD)
    osd_config_t load_osd_config() const;
    int32_t store_osd_config(const osd_config_t& config);
    bool load_osd_elements_config(osd_elements_config_t & config) const;
    int32_t store_osd_elements_config(const osd_elements_config_t& config);
#endif
#if defined(USE_GPS)
    gps_config_t load_gps_config() const;
    int32_t store_gps_config(const gps_config_t& config);
#endif
#if defined(USE_ALTITUDE_HOLD)
    autopilot_config_t load_autopilot_config() const;
    int32_t store_autopilot_config(const autopilot_config_t& config);
    position_hold_config_t load_autopilot_position_hold_config() const;
    int32_t store_autopilot_position_hold_config(const position_hold_config_t& config);
    altitude_hold_config_t load_altitude_hold_config() const;
    int32_t store_altitude_hold_config(const altitude_hold_config_t& config);
#endif

private:
#if defined(USE_FLASH_KLV)
    FlashKLV _flashKLV;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    mutable Preferences _preferences {};
#endif
    uint8_t _current_pid_profile_index {0};
    uint8_t _current_rate_profile_index {0};
};
