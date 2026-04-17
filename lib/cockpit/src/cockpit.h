#pragma once

#include "features.h"
#include "msp_box.h"
#include "rates.h"
#include "rc_adjustments.h"
#include "rx.h"
#include "targets.h"

#include <cassert>
#include <cockpit_base.h>


class Autopilot;
class Debug;
class Blackbox;
class FlightController;
class OSD;
class MotorMixerBase;
class RcModes;

struct receiver_context_t {
    RcModes& rc_modes;
    FlightController& flight_controller;
    MotorMixerBase& motor_mixer;
    Debug& debug;
    Blackbox* blackbox;
    OSD* osd;
};


static constexpr uint32_t ARMING_DISABLED_NO_GYRO         = (1U << 0U);
static constexpr uint32_t  ARMING_DISABLED_FAILSAFE        = (1U << 1U);
static constexpr uint32_t  ARMING_DISABLED_RX_FAILSAFE     = (1U << 2U);
static constexpr uint32_t  ARMING_DISABLED_NOT_DISARMED    = (1U << 3U);
static constexpr uint32_t  ARMING_DISABLED_BOXFAILSAFE     = (1U << 4U);
static constexpr uint32_t  ARMING_DISABLED_RUNAWAY_TAKEOFF = (1U << 5U);
static constexpr uint32_t  ARMING_DISABLED_CRASH_DETECTED  = (1U << 6U);
static constexpr uint32_t  ARMING_DISABLED_THROTTLE        = (1U << 7U);
static constexpr uint32_t  ARMING_DISABLED_ANGLE           = (1U << 8U);
static constexpr uint32_t  ARMING_DISABLED_BOOT_GRACE_TIME = (1U << 9U);
static constexpr uint32_t  ARMING_DISABLED_NOPREARM        = (1U << 10U);
static constexpr uint32_t  ARMING_DISABLED_LOAD            = (1U << 11U);
static constexpr uint32_t  ARMING_DISABLED_CALIBRATING     = (1U << 12U);
static constexpr uint32_t  ARMING_DISABLED_CLI             = (1U << 13U);
static constexpr uint32_t  ARMING_DISABLED_CMS_MENU        = (1U << 14U);
static constexpr uint32_t  ARMING_DISABLED_BST             = (1U << 15U);
static constexpr uint32_t  ARMING_DISABLED_MSP             = (1U << 16U);
static constexpr uint32_t  ARMING_DISABLED_PARALYZE        = (1U << 17U);
static constexpr uint32_t  ARMING_DISABLED_GPS             = (1U << 18U);
static constexpr uint32_t  ARMING_DISABLED_RESC            = (1U << 19U);
static constexpr uint32_t  ARMING_DISABLED_DSHOT_TELEM     = (1U << 20U);
static constexpr uint32_t  ARMING_DISABLED_REBOOT_REQUIRED = (1U << 21U);
static constexpr uint32_t  ARMING_DISABLED_DSHOT_BITBANG   = (1U << 22U);
static constexpr uint32_t  ARMING_DISABLED_ACC_CALIBRATION = (1U << 23U);
static constexpr uint32_t  ARMING_DISABLED_MOTOR_PROTOCOL  = (1U << 24U);
static constexpr uint32_t  ARMING_DISABLED_CRASHFLIP       = (1U << 25U);
static constexpr uint32_t  ARMING_DISABLED_ARM_SWITCH      = (1U << 26U); // Needs to be the last element, since it's always activated if one of the others is active when arming

static constexpr uint32_t  ARMING_DISABLE_FLAGS_COUNT = 27;

// MSP compatible failsafe parameters
struct failsafe_config_t {
    uint16_t throttle_pwm;
    uint16_t throttle_low_delay_deciseconds;
    uint16_t recovery_delay_deciseconds; // time of valid rx data needed to allow recovery from failsafe and re-arming
    uint8_t delay_deciseconds;
    uint8_t landing_time_seconds; // time allowed in landing phase before disarm
    uint8_t procedure;
    uint8_t switch_mode;
    uint8_t stick_threshold_percent; // _stick deflection percentage to exit GPS Rescue procedure
};

class Cockpit : public CockpitBase {
public:
    Cockpit(Autopilot& autopilot, const RcAdjustments::adjustment_configs_t* defaultAdjustment_configs);
public:
    static constexpr uint8_t FAILSAFE_DISARMED = 0;
    static constexpr uint8_t FAILSAFE_IDLE = 1;
    static constexpr uint8_t FAILSAFE_RX_LOSS_DETECTED = 2;
    static constexpr uint8_t FAILSAFE_RX_LOSS_MONITORING = 3;
    static constexpr uint8_t FAILSAFE_RX_LOSS_RECOVERED = 4;
    static constexpr uint8_t FAILSAFE_LANDING = 5;
    static constexpr uint8_t FAILSAFE_LANDED = 6;
    static constexpr uint8_t FAILSAFE_GPS_RESCUE = 7;
    struct failsafe_t {
        uint32_t tick_count; //<! failsafe counter, so the vehicle doesn't fly away if it looses contact with the transmitter (for example by going out of range)
        uint32_t tick_count_threshold;
        uint32_t tick_count_switch_off_threshold;
        uint8_t phase;
    };
    enum failsafe_procedure_e {
        FAILSAFE_PROCEDURE_DROP_IT = 0,
        FAILSAFE_PROCEDURE_AUTO_LANDING,
        FAILSAFE_PROCEDURE_GPS_RESCUE,
        FAILSAFE_PROCEDURE_COUNT
    };
    enum failsafe_switch_mode_e {
        FAILSAFE_SWITCH_MODE_STAGE1 = 0,
        FAILSAFE_SWITCH_MODE_STAGE2,
        FAILSAFE_SWITCH_MODE_KILL,
    };
    // arming flags
    static constexpr uint32_t ARMED = 0x01;
    static constexpr uint32_t WAS_EVER_ARMED = 0x02;
    static constexpr uint32_t WAS_ARMED_WITH_PREARM = 0x04;
    // flight mode flags
    enum log2_flight_mode_flag_e {
        LOG2_ANGLE_MODE         = 0,
        LOG2_HORIZON_MODE       = 1,
        LOG2_MAG_MODE           = 2,
        LOG2_ALTITUDE_HOLD_MODE = 3,
//        LOG2_GPS_HOME_MODE      = 4,
        LOG2_POSITION_HOLD_MODE = 5,
        LOG2_HEADFREE_MODE      = 6,
        LOG2_CHIRP_MODE         = 7,
        LOG2_PASSTHRU_MODE      = 8,
//        LOG2_RANGEFINDER_MODE   = 9,
        LOG2_FAILSAFE_MODE      = 10,
        LOG2_GPS_RESCUE_MODE    = 11,
        FLIGHT_MODE_FLAG_COUNT = 12
    };
    static constexpr uint32_t ANGLE_MODE      = 1U << LOG2_ANGLE_MODE;
    static constexpr uint32_t HORIZON_MODE    = 1U << LOG2_HORIZON_MODE;
    static constexpr uint32_t MAG_MODE        = 1U << LOG2_MAG_MODE;
    static constexpr uint32_t ALTITUDE_HOLD_MODE = 1U << LOG2_ALTITUDE_HOLD_MODE;
//    static constexpr uint32_t GPS_HOME_MODE   = 1U << LOG2_GPS_HOME_MODE;
    static constexpr uint32_t POSITION_HOLD_MODE = 1U << LOG2_POSITION_HOLD_MODE;
    static constexpr uint32_t HEADFREE_MODE   = 1U << LOG2_HEADFREE_MODE;
    static constexpr uint32_t CHIRP_MODE      = 1U << LOG2_CHIRP_MODE;
    static constexpr uint32_t PASSTHRU_MODE   = 1U << LOG2_PASSTHRU_MODE;
//    static constexpr uint32_t RANGEFINDER_MODE= 1U << LOG2_RANGEFINDER_MODE;
    static constexpr uint32_t FAILSAFE_MODE   = 1U << LOG2_FAILSAFE_MODE;
    static constexpr uint32_t GPS_RESCUE_MODE = 1U << LOG2_GPS_RESCUE_MODE;
public:
    virtual void update_controls(uint32_t tick_count, const ReceiverBase& receiver, receiver_context_t& ctx) override;
    virtual void check_failsafe(uint32_t tick_count, receiver_context_t& ctx) override;

    const Autopilot& get_autopilot() const { return _autopilot; }
    Autopilot& get_autopilot_mutable() { return _autopilot; }

    void handle_arming_switch(FlightController& flight_controller, MotorMixerBase& motor_mixer, Blackbox* blackbox, const ReceiverBase& receiver, const RcModes& rc_modes, const Debug& debug);

    bool feature_is_enabled(uint32_t mask) const { return _features.is_enabled(mask); }
    uint32_t enabled_features() const { return _features.enabled_features(); }
    void set_features(uint32_t features) { _features.set(features); }
    features_config_t get_features_config() const { return features_config_t { .enabled_features = enabled_features() }; }

    bool is_armed() const;
    bool was_ever_armed() const;
    void set_armed(FlightController& flight_controller, MotorMixerBase& motor_mixer);
    void set_disarmed(FlightController& flight_controller, MotorMixerBase& motor_mixer);
    void set_arming_disabled_flag(uint32_t flag);
    void clear_arming_disabled_flag(uint32_t flag);
    uint32_t get_arming_disabled_flags() const { return _arming_disabled_flags; }
    uint32_t get_flight_mode_flags() const;
    void set_flight_mode_flag(uint8_t flag) { if (flag < FLIGHT_MODE_FLAG_COUNT) { _flight_mode_flags.set(flag); } } // for testing

    uint8_t get_failsafe_phase() const { return _failsafe.phase; }

    const failsafe_config_t& get_failsafe_config() const { return _failsafe_config; }
    void set_failsafe_config(const failsafe_config_t& failsafe_config);
    bool gps_rescue_is_configured() const;

    const rx_config_t& get_rx_config() const { return _rx_config; }
    void set_rx_config(const rx_config_t& rx_config);
    const rx_failsafe_channel_configs_t& get_rx_failsafe_channel_configs() const { return _rx_failsafe_channel_configs; }
    void set_rx_failsafe_channel_configs(const rx_failsafe_channel_configs_t& rx_failsafe_channel_configs);

    const rates_t& get_rates() const { return _rates; }
    rates_t& get_rates() { return _rates; }
    void set_rates(const rates_t& rates);
    void set_rates_to_pass_through();
    float apply_rates(size_t axis, float rcCommand) const;
    float map_throttle(float throttle) const;

#if defined(USE_RC_ADJUSTMENTS)
    const RcAdjustments& get_rc_adjustments() const { return _rc_adjustments; }
    RcAdjustments& get_rc_adjustments() { return _rc_adjustments; }
#endif

    void set_record_to_blackbox_when_armed(bool record_to_blackbox_when_armed) { _record_to_blackbox_when_armed = record_to_blackbox_when_armed; }
    void start_blackbox_recording(Blackbox* blackbox, FlightController& flight_controller, const MotorMixerBase& motor_mixer, const Debug& debug);
    void stop_blackbox_recording(Blackbox* blackbox, FlightController& flight_controller);

    bool get_box_id_state(uint8_t box_id, const RcModes& rc_modes) const;
    size_t pack_flight_mode_flags(std::bitset<MSP_BOX_COUNT>& flight_mode_flags, const RcModes& rc_modes) const;
    void serialize_box_reply_box_name(StreamBufWriter& dst, size_t page) const { _msp_box.serialize_box_reply_box_name(dst, page); }
    void serialize_box_reply_permanent_id(StreamBufWriter& dst, size_t page) const { _msp_box.serialize_box_reply_permanent_id(dst, page); }
private:
    MspBox _msp_box;
    Features _features {};
    Autopilot& _autopilot;
    rates_t _rates {
        .rate_limits = { rates_t::LIMIT_MAX, rates_t::LIMIT_MAX, rates_t::LIMIT_MAX },
        .rc_rates = { 100, 100, 100 },
        .rc_expos = { 0, 0, 0 },
        .rates = { 0, 0, 0 },
        .throttle_midpoint = 50,
        .throttle_expo = 0,
        .throttle_limit_type = rates_t::THROTTLE_LIMIT_TYPE_OFF,
        .throttle_limit_percent = 100
    };
    uint32_t _arming_flags {};
    uint32_t _arming_disabled_flags {};
    std::bitset<FLIGHT_MODE_FLAG_COUNT> _flight_mode_flags {};
    failsafe_config_t _failsafe_config {};
    rx_config_t _rx_config {};
    rx_failsafe_channel_configs_t _rx_failsafe_channel_configs {};
    // failsafe handling
    failsafe_t _failsafe {
        .tick_count = 0,
        .tick_count_threshold = 1500,
        .tick_count_switch_off_threshold = 5000,
        .phase = FAILSAFE_DISARMED
    };
    bool _record_to_blackbox_when_armed {false};
    bool _reboot_required {false};
    bool _on_off_switch_pressed {false}; // on/off switch debouncing
    bool _cli_mode {false};
    bool _gps_rescue_configured {false};
#if defined(USE_RC_ADJUSTMENTS)
    RcAdjustments _rc_adjustments;
#endif
public:
    static constexpr std::array<uint8_t, MspBox::BOX_ID_FLIGHTMODE_COUNT> _box_idToFlightModeMap = {{
        /*[BOX_ARM]*/           0, // not used
        /*[BOX_ANGLE]*/         LOG2_ANGLE_MODE,
        /*[BOX_HORIZON]*/       LOG2_HORIZON_MODE,
        /*[BOX_MAG]*/           LOG2_MAG_MODE,
        /*[BOX_ALTITUDE_HOLD]*/ LOG2_ALTITUDE_HOLD_MODE,
        /*[BOX_POSITION_HOLD]*/ LOG2_POSITION_HOLD_MODE,
        /*[BOX_HEADFREE]*/      LOG2_HEADFREE_MODE,
        /*[BOX_CHIRP]*/         LOG2_CHIRP_MODE,
        /*[BOX_PASSTHRU]*/      LOG2_PASSTHRU_MODE,
        /*[BOX_FAILSAFE]*/      LOG2_FAILSAFE_MODE,
        /*[BOX_GPS_RESCUE]*/    LOG2_GPS_RESCUE_MODE
    }};
};
