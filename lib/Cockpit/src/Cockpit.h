#pragma once

#include "Features.h"
#include "RC_Adjustments.h"
#include "RC_Modes.h"
#include "RX.h"
#include "Rates.h"
#include "Targets.h"

#include <CockpitBase.h>

#include <cassert>

class Autopilot;
class Debug;
class Blackbox;
class FlightController;
class IMU_Filters;
class ReceiverBase;

enum arming_disabled_flags_e {
    ARMING_DISABLED_NO_GYRO         = (1U << 0U),
    ARMING_DISABLED_FAILSAFE        = (1U << 1U),
    ARMING_DISABLED_RX_FAILSAFE     = (1U << 2U),
    ARMING_DISABLED_NOT_DISARMED    = (1U << 3U),
    ARMING_DISABLED_BOXFAILSAFE     = (1U << 4U),
    ARMING_DISABLED_RUNAWAY_TAKEOFF = (1U << 5U),
    ARMING_DISABLED_CRASH_DETECTED  = (1U << 6U),
    ARMING_DISABLED_THROTTLE        = (1U << 7U),
    ARMING_DISABLED_ANGLE           = (1U << 8U),
    ARMING_DISABLED_BOOT_GRACE_TIME = (1U << 9U),
    ARMING_DISABLED_NOPREARM        = (1U << 10U),
    ARMING_DISABLED_LOAD            = (1U << 11U),
    ARMING_DISABLED_CALIBRATING     = (1U << 12U),
    ARMING_DISABLED_CLI             = (1U << 13U),
    ARMING_DISABLED_CMS_MENU        = (1U << 14U),
    ARMING_DISABLED_BST             = (1U << 15U),
    ARMING_DISABLED_MSP             = (1U << 16U),
    ARMING_DISABLED_PARALYZE        = (1U << 17U),
    ARMING_DISABLED_GPS             = (1U << 18U),
    ARMING_DISABLED_RESC            = (1U << 19U),
    ARMING_DISABLED_DSHOT_TELEM     = (1U << 20U),
    ARMING_DISABLED_REBOOT_REQUIRED = (1U << 21U),
    ARMING_DISABLED_DSHOT_BITBANG   = (1U << 22U),
    ARMING_DISABLED_ACC_CALIBRATION = (1U << 23U),
    ARMING_DISABLED_MOTOR_PROTOCOL  = (1U << 24U),
    ARMING_DISABLED_CRASHFLIP       = (1U << 25U),
    ARMING_DISABLED_ARM_SWITCH      = (1U << 26U), // Needs to be the last element, since it's always activated if one of the others is active when arming
};

enum { ARMING_DISABLE_FLAGS_COUNT = 27 };


class Cockpit : public CockpitBase {
public:
    Cockpit(RcModes& rc_modes, FlightController& flightController, Autopilot& autopilot, IMU_Filters& imuFilters, Debug& debug, const RC_Adjustments::adjustment_configs_t* defaultAdjustmentConfigs);
public:
    enum failsafe_phase_e {
        FAILSAFE_DISARMED = 0,
        FAILSAFE_IDLE,
        FAILSAFE_RX_LOSS_DETECTED,
        FAILSAFE_RX_LOSS_MONITORING,
        FAILSAFE_RX_LOSS_RECOVERED,
        FAILSAFE_LANDING,
        FAILSAFE_LANDED,
        FAILSAFE_GPS_RESCUE
    };
    struct failsafe_t {
        failsafe_phase_e phase;
        uint32_t tickCount; //<! failsafe counter, so the vehicle doesn't fly away if it looses contact with the transmitter (for example by going out of range)
        uint32_t tickCountThreshold;
        uint32_t tickCountSwitchOffThreshold;
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
    // MSP compatible failsafe parameters
    struct failsafe_config_t {
        uint16_t throttle_pwm;
        uint16_t throttle_low_delay_deciseconds;
        uint16_t recovery_delay_deciseconds; // time of valid rx data needed to allow recovery from failsafe and re-arming
        uint8_t delay_deciseconds;
        uint8_t landing_time_seconds; // time allowed in landing phase before disarm
        uint8_t procedure;
        uint8_t switch_mode;
        uint8_t stick_threshold_percent; // Stick deflection percentage to exit GPS Rescue procedure
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
    void setBlackbox(Blackbox& blackbox) { _blackbox = &blackbox; }
    void setOSD(OSD& osd) { _osd = &osd; }

    const Autopilot& getAutopilot() const { return _autopilot; }
    Autopilot& getAutopilotMutable() { return _autopilot; }
    const FlightController& getFlightController() const { return _flightController; }
    FlightController& getFlightControllerMutable() { return _flightController; }

    void handleArmingSwitch(FlightController& flightController, const ReceiverBase& receiver, const RcModes& rcModes);
    virtual void update_controls(uint32_t tickCount, ReceiverBase& receiver) override;

    bool featureIsEnabled(uint32_t mask) const { return _features.isEnabled(mask); }
    uint32_t enabledFeatures() const { return _features.enabledFeatures(); }
    void setFeatures(uint32_t features) { _features.set(features); }
    Features::config_t getFeaturesConfig() const { return Features::config_t { .enabledFeatures = enabledFeatures() }; }

    bool isArmed() const;
    bool wasEverArmed() const;
    void setArmed(FlightController& flightController);
    void setDisarmed(FlightController& flightController);
    void setArmingDisabledFlag(arming_disabled_flags_e flag);
    void clearArmingDisabledFlag(arming_disabled_flags_e flag);
    uint32_t getArmingDisableFlags() const { return _armingDisabledFlags; }
    uint32_t getFlightModeFlags() const;
    void setFlightModeFlag(uint8_t flag) { if (flag < FLIGHT_MODE_FLAG_COUNT) { _flightModeFlags.set(flag); } } // for testing

    virtual void check_failsafe(uint32_t tickCount) override;
    failsafe_phase_e getFailsafePhase() const { return _failsafe.phase; }

    const failsafe_config_t& getFailsafeConfig() const { return _failsafeConfig; }
    void setFailsafeConfig(const failsafe_config_t& failsafeConfig);
    bool gpsRescueIsConfigured() const;

    const RX::config_t& getRX_Config() const { return _rxConfig; }
    void setRX_Config(const RX::config_t& rxConfig);
    const RX::failsafe_channel_configs_t& getRX_FailsafeChannelConfigs() const { return _rxFailsafeChannelConfigs; }
    void setRX_FailsafeChannelConfigs(const RX::failsafe_channel_configs_t& rxFailsafeChannelConfigs);

    const rates_t& getRates() const { return _rates; }
    rates_t& getRates() { return _rates; }
    void setRates(const rates_t& rates, FlightController& flightController);
    void setRatesToPassThrough();
    float applyRates(size_t axis, float rcCommand) const;
    float mapThrottle(float throttle) const;

#if defined(USE_RC_ADJUSTMENTS)
    const RC_Adjustments& getRC_Adjustments() const { return _rcAdjustments; }
    RC_Adjustments& getRC_Adjustments() { return _rcAdjustments; }
#endif

    void setRecordToBlackboxWhenArmed(bool recordToBlackboxWhenArmed) { _recordToBlackboxWhenArmed = recordToBlackboxWhenArmed; }
    void startBlackboxRecording(FlightController& flightController);
    void stopBlackboxRecording(FlightController& flightController);

    bool getBoxIdState(uint8_t boxId, const RcModes& rcModes) const;
    size_t packFlightModeFlags(MspBox::bitset_t& flightModeFlags, const RcModes& rcModes) const;
    void serialize_box_reply_box_name(StreamBufWriter& dst, size_t page) const { _mspBox.serialize_box_reply_box_name(dst, page); }
    void serialize_box_reply_permanent_id(StreamBufWriter& dst, size_t page) const { _mspBox.serialize_box_reply_permanent_id(dst, page); }
private:
    MspBox _mspBox;
    RcModes& _rc_modes;
    Features _features {};
    FlightController& _flightController;
    Autopilot& _autopilot;
    IMU_Filters& _imuFilters;
    Debug& _debug;
    rates_t _rates {
        .rateLimits = { rates_t::LIMIT_MAX, rates_t::LIMIT_MAX, rates_t::LIMIT_MAX },
        .rcRates = { 100, 100, 100 },
        .rcExpos = { 0, 0, 0 },
        .rates = { 0, 0, 0 },
        .throttleMidpoint = 50,
        .throttleExpo = 0,
        .throttleLimitType = rates_t::THROTTLE_LIMIT_TYPE_OFF,
        .throttleLimitPercent = 100
    };
    Blackbox* _blackbox {nullptr};
    OSD* _osd {nullptr};
    uint32_t _armingFlags {};
    uint32_t _armingDisabledFlags {};
    std::bitset<FLIGHT_MODE_FLAG_COUNT> _flightModeFlags {};
    failsafe_config_t _failsafeConfig {};
    RX::config_t _rxConfig {};
    RX::failsafe_channel_configs_t _rxFailsafeChannelConfigs {};
    // failsafe handling
    failsafe_t _failsafe {
        .phase = FAILSAFE_DISARMED,
        .tickCount = 0,
        .tickCountThreshold = 1500,
        .tickCountSwitchOffThreshold = 5000
    };
    bool _recordToBlackboxWhenArmed { false };
    bool _rebootRequired {false};
    bool _onOffSwitchPressed {false}; // on/off switch debouncing
    bool _cliMode {false};
    bool _gpsRescueConfigured {false};
#if defined(USE_RC_ADJUSTMENTS)
    RC_Adjustments _rcAdjustments;
#endif
public:
    static constexpr std::array<uint8_t, MspBox::BOX_ID_FLIGHTMODE_COUNT> BoxIdToFlightModeMap = {{
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
