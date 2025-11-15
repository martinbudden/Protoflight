#pragma once

#include <CockpitBase.h>
#include <Features.h>

#include <array>
#include <cstddef>
#include <cstdint>

class Autopilot;
class Debug;
class Blackbox;
class FlightController;
class IMU_Filters;
class NonVolatileStorage;
class ReceiverBase;


class Cockpit : public CockpitBase {
public:
    enum { ROLL = 0, PITCH = 1, YAW = 2, AXIS_COUNT = 3 };
    enum { RATE_LIMIT_MAX = 1998 };
    enum throttleLimitType_e { THROTTLE_LIMIT_TYPE_OFF = 0, THROTTLE_LIMIT_TYPE_SCALE, THROTTLE_LIMIT_TYPE_CLIP, THROTTLE_LIMIT_TYPE_COUNT };
    enum ratesType_e { RATES_TYPE_BETAFLIGHT = 0, RATES_TYPE_RACEFLIGHT, RATES_TYPE_KISS, RATES_TYPE_ACTUAL, RATES_TYPE_QUICK, RATES_TYPE_COUNT } ;
    struct rates_t {
        std::array<uint16_t, AXIS_COUNT> rateLimits;
        std::array<uint8_t, AXIS_COUNT> rcRates; // center sensitivity
        std::array<uint8_t, AXIS_COUNT> rcExpos; // movement sensitivity, nonlinear
        std::array<uint8_t, AXIS_COUNT> rates; // movement sensitivity, linear
        uint8_t throttleMidpoint; // not used
        uint8_t throttleExpo;
        uint8_t throttleLimitType; // not used
        uint8_t throttleLimitPercent; // Sets the maximum pilot commanded throttle limit
        //uint8_t ratesType; // not used
    };
    enum failsafe_phase_e {
        FAILSAFE_IDLE = 0,
        FAILSAFE_RX_LOSS_DETECTED,
        FAILSAFE_LANDING,
        FAILSAFE_LANDED,
        FAILSAFE_RX_LOSS_MONITORING,
        FAILSAFE_RX_LOSS_RECOVERED,
        FAILSAFE_GPS_RESCUE
    };
    enum failsafe_procedure_e {
        FAILSAFE_PROCEDURE_AUTO_LANDING = 0,
        FAILSAFE_PROCEDURE_DROP_IT,
        FAILSAFE_PROCEDURE_GPS_RESCUE,
        FAILSAFE_PROCEDURE_COUNT   // must be last
    };
    enum { PERIOD_RX_DATA_RECOVERY_MS = 100 };
    // MSP compatible failsafe parameters
    struct failsafe_t {
        uint8_t delay;
        uint8_t landing_time;
        uint8_t switch_mode;
        uint8_t procedure;
        uint16_t throttle;
        uint16_t throttle_low_delay;
        uint16_t failsafe_recovery_delay;
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
        LOG2_ALT_HOLD_MODE      = 3,
        LOG2_GPS_HOME_MODE      = 4,
        LOG2_POS_HOLD_MODE      = 5,
        LOG2_HEADFREE_MODE      = 6,
        LOG2_CHIRP_MODE         = 7,
        LOG2_PASSTHRU_MODE      = 8,
        LOG2_RANGEFINDER_MODE   = 9,
        LOG2_FAILSAFE_MODE      = 10,
        LOG2_GPS_RESCUE_MODE    = 11
    };
    static constexpr uint32_t ANGLE_MODE      = 1U << LOG2_ANGLE_MODE;
    static constexpr uint32_t HORIZON_MODE    = 1U << LOG2_HORIZON_MODE;
    static constexpr uint32_t MAG_MODE        = 1U << LOG2_MAG_MODE;
    static constexpr uint32_t ALT_HOLD_MODE   = 1U << LOG2_ALT_HOLD_MODE;
    static constexpr uint32_t GPS_HOME_MODE   = 1U << LOG2_GPS_HOME_MODE;
    static constexpr uint32_t POS_HOLD_MODE   = 1U << LOG2_POS_HOLD_MODE;
    static constexpr uint32_t HEADFREE_MODE   = 1U << LOG2_HEADFREE_MODE;
    static constexpr uint32_t CHIRP_MODE      = 1U << LOG2_CHIRP_MODE;
    static constexpr uint32_t PASSTHRU_MODE   = 1U << LOG2_PASSTHRU_MODE;
    static constexpr uint32_t RANGEFINDER_MODE= 1U << LOG2_RANGEFINDER_MODE;
    static constexpr uint32_t FAILSAFE_MODE   = 1U << LOG2_FAILSAFE_MODE;
    static constexpr uint32_t GPS_RESCUE_MODE = 1U << LOG2_GPS_RESCUE_MODE;
public:
    Cockpit(ReceiverBase& receiver, FlightController& flightController, Autopilot& autopilot, IMU_Filters& imuFilters,  Debug& _debug, NonVolatileStorage& nvs);
    void setBlackbox(Blackbox& blackbox) { _blackbox = &blackbox; }

    const Autopilot& getAutopilot() const { return _autopilot; }
    FlightController& getFlightController() { return _flightController; }

    uint8_t getCurrentPidProfileIndex() const { return _currentPidProfileIndex; }
    void setCurrentPidProfileIndex(uint8_t currentPidProfileIndex);

    uint8_t getCurrentRateProfileIndex() const { return _currentRateProfileIndex; }
    void setCurrentRateProfileIndex(uint8_t currentRateProfileIndex);
    void storeAllToNonVolatileStorage();
    void handleOnOffSwitch();
    virtual void updateControls(const controls_t& controls) override;

    bool featureIsEnabled(uint32_t mask) const { return _features.featureIsEnabled(mask); }
    uint32_t enabledFeatures() const { return _features.enabledFeatures(); }
    void setFeatures(uint32_t features) { _features.setFeatures(features); }

    bool isArmed() const;
    bool wasEverArmed() const;
    void setArmed();
    void setDisarmed();
    bool isFlightModeFlagSet(uint32_t flightModeFlag) const;
    void setFlightModeFlag(uint32_t flightModeFlag);
    void clearFlightModeFlag(uint32_t flightModeFlag);
    uint32_t getFlightModeFlags() const;
    bool isRcModeActive(uint8_t rcMode) const;

    virtual void checkFailsafe(uint32_t tickCount) override;
    const failsafe_t& getFailsafe() const { return _failsafe; }
    void setFailsafe(const failsafe_t& failsafe);
    failsafe_phase_e getFailsafePhase() const { return _failsafePhase; }

    const rates_t& getRates() const { return _rates; }
    void setRates(const rates_t& rates) { _rates = rates; }
    void setRatesToPassThrough();
    float applyRates(size_t axis, float rcCommand) const;
    float mapThrottle(float throttle) const;
private:
    Features _features;
    FlightController& _flightController;
    Autopilot& _autopilot;
    IMU_Filters& _imuFilters;
    Debug& _debug;
    NonVolatileStorage& _nvs;
    rates_t _rates {
        .rateLimits = { RATE_LIMIT_MAX, RATE_LIMIT_MAX, RATE_LIMIT_MAX },
        .rcRates = { 100, 100, 100 },
        .rcExpos = { 0, 0, 0 },
        .rates = { 0, 0, 0 },
        .throttleMidpoint = 50,
        .throttleExpo = 0,
        .throttleLimitType = THROTTLE_LIMIT_TYPE_OFF,
        .throttleLimitPercent = 100
    };
    Blackbox* _blackbox {nullptr};
    int32_t _onOffSwitchPressed {false}; // on/off switch debouncing
    float _maxRollAngleDegrees { 60.0F }; // used for angle mode
    float _maxPitchAngleDegrees { 60.0F }; // used for angle mode
    uint32_t _armingFlags {};
    uint32_t _flightModeFlags {};
    // failsafe handling
    failsafe_phase_e _failsafePhase {FAILSAFE_IDLE};
    failsafe_t _failsafe {};
    int32_t _receiverInUse {false};
    uint32_t _failsafeTickCount {0}; //!< failsafe counter, so the vehicle doesn't fly away if it looses contact with the transmitter (for example by going out of range)
    uint32_t _failsafeTickCountThreshold {1500};
    uint32_t _failsafeTickCountSwitchOffThreshold {5000};
    uint8_t _currentPidProfileIndex {0};
    uint8_t _currentRateProfileIndex {0};
};
