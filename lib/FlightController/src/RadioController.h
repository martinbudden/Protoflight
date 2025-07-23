#pragma once

#include <RadioControllerBase.h>

#include <array>
#include <cstddef>
#include <cstdint>

class FlightController;

class RadioController : public RadioControllerBase {
public:
    explicit RadioController(ReceiverBase& receiver);
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
        uint8_t ratesType; // not used
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
    // MSP compatible failsafe parameters
    struct failsafe_t {
        uint8_t delay;
        uint8_t landing_time;
        uint8_t switch_mode;
        uint8_t procedure;
        uint16_t throttle;
        uint16_t throttle_low_delay;
    };
public:
    void setFlightController(FlightController* flightController) { _flightController = flightController; }

    virtual void updateControls(const controls_t& controls) override;
    virtual uint32_t getFailsafePhase() const override;

    virtual void checkFailsafe(uint32_t tickCount) override;
    const failsafe_t& getFailsafe() const { return _failsafe; }
    void setFailsafe(const failsafe_t& failsafe);

    rates_t getRates() const { return _rates; }
    void setRates(const rates_t& rates) { _rates = rates; }
    void setRatesToPassThrough();
    float applyRates(size_t axis, float rcCommand) const;
    float mapThrottle(float throttle) const;
private:
    FlightController* _flightController {};
    rates_t _rates;
    int32_t _onOffSwitchPressed {false}; // on/off switch debouncing
    float _maxRollAngleDegrees { 60.0F }; // used for angle mode
    float _maxPitchAngleDegrees { 60.0F }; // used for angle mode
    // failsafe handling
    failsafe_phase_e _failsafePhase {FAILSAFE_IDLE};
    failsafe_t _failsafe {};
    int32_t _receiverInUse {false};
    uint32_t _failsafeTickCount {0}; //<! failsafe counter, so the vehicle doesn't fly away if it looses contact with the transmitter (for example by going out of range)
    uint32_t _failsafeTickCountThreshold {1500};
    uint32_t _failsafeTickCountSwitchOffThreshold {5000};
};
