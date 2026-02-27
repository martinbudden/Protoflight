#include "Autopilot.h"
#include "Cockpit.h"
#include "Defaults.h"
#include "FlightController.h"

#include <RC_Modes.h>

#include <ahrs.h>
#include <ahrs_message_queue.h>
#include <receiver_virtual.h>
#include <sensor_fusion.h>


#include <unity.h>

// NOLINTBEGIN(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif

static AhrsMessageQueue ahrsMessageQueue;
static FlightController flightController(AHRS_TASK_INTERVAL_MICROSECONDS);
static Autopilot autopilot(ahrsMessageQueue);
static ReceiverVirtual receiver;
static Blackbox* blackbox =  nullptr;


static const rates_t cockpitRates {
    .rateLimits = { rates_t::LIMIT_MAX, rates_t::LIMIT_MAX, rates_t::LIMIT_MAX},
    .rcRates = { 7, 7, 7 },
    .rcExpos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttleMidpoint = 50,
    .throttleExpo = 0,
    .throttleLimitType = rates_t::THROTTLE_LIMIT_TYPE_OFF,
    .throttleLimitPercent = 100,
    //.ratesType = rates_t::RATES_TYPE_ACTUAL
};

void setUp() {
}

void tearDown() {
}

void test_cockpit()
{
    static Cockpit cockpit(autopilot, nullptr);
    cockpit.setRates(cockpitRates);

    rates_t rates = cockpit.getRates();

    // rates.rcRates apply a linear scale factor
    rates.rcRates = {100, 100, 100};
    rates.rcExpos = {0, 0, 0};
    rates.rates = {0, 0, 0};
    //rates.ratesType = rates_t::RATES_TYPE_ACTUAL;
    cockpit.setRates(rates);

    float roll = cockpit.applyRates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0, roll);

    // rates.rates apply a nonlinear scale factor
    rates.rcRates = {0, 0, 0};
    rates.rates = {60, 60, 60};
    cockpit.setRates(rates);

    roll = cockpit.applyRates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(37.5F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.50F);
    TEST_ASSERT_EQUAL_FLOAT(150.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(337.5F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(600.0F, roll); // 100.0F / (1.0F - 0.60F)
}

void test_cockpit_passthrough()
{
    static Cockpit cockpit(autopilot, nullptr);

    float roll = cockpit.applyRates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
}

void test_cockpit_set_passthrough()
{
    static Cockpit cockpit(autopilot, nullptr);

    cockpit.setRatesToPassThrough();

    float roll = cockpit.applyRates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
}

void test_cockpit_defaults()
{
    static Cockpit cockpit(autopilot, nullptr);
    cockpit.setRates(cockpitRates);

    const rates_t rates = cockpit.getRates();

    TEST_ASSERT_EQUAL(rates_t::LIMIT_MAX, rates.rateLimits[0]);
    TEST_ASSERT_EQUAL(rates_t::LIMIT_MAX, rates.rateLimits[1]);
    TEST_ASSERT_EQUAL(rates_t::LIMIT_MAX, rates.rateLimits[2]);
    TEST_ASSERT_EQUAL(7, rates.rcRates[0]);
    TEST_ASSERT_EQUAL(7, rates.rcRates[1]);
    TEST_ASSERT_EQUAL(7, rates.rcRates[2]);
    TEST_ASSERT_EQUAL(0, rates.rcExpos[0]);
    TEST_ASSERT_EQUAL(0, rates.rcExpos[1]);
    TEST_ASSERT_EQUAL(0, rates.rcExpos[2]);
    TEST_ASSERT_EQUAL(67, rates.rates[0]);
    TEST_ASSERT_EQUAL(67, rates.rates[1]);
    TEST_ASSERT_EQUAL(67, rates.rates[2]);
    TEST_ASSERT_EQUAL(50, rates.throttleMidpoint);
    TEST_ASSERT_EQUAL(0, rates.throttleExpo);
    TEST_ASSERT_EQUAL(rates_t::THROTTLE_LIMIT_TYPE_OFF, rates.throttleLimitType);
    TEST_ASSERT_EQUAL(100, rates.throttleLimitPercent);
    //TEST_ASSERT_EQUAL(rates_t::RATES_TYPE_ACTUAL, rates.ratesType);

    float roll = cockpit.applyRates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(55.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(185.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(390.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(670.0F, roll);
}

void test_cockpit_constrain()
{
    static Cockpit cockpit(autopilot, nullptr);
    cockpit.setRates(cockpitRates);

    rates_t rates = cockpit.getRates(); // NOLINT(misc-const-correctness)
    rates.rcRates = {200, 200, 200};
    cockpit.setRates(rates);

    float roll = cockpit.applyRates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(1500.0F, roll);
    roll = cockpit.applyRates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1998.0F, roll);
}

void test_cockpit_throttle()
{
    static Cockpit cockpit(autopilot, nullptr);
    cockpit.setRates(cockpitRates);

    float throttle = cockpit.mapThrottle(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    throttle = cockpit.mapThrottle(0.25F);
    TEST_ASSERT_EQUAL_FLOAT(0.25F, throttle);
    throttle = cockpit.mapThrottle(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, throttle);
    throttle = cockpit.mapThrottle(0.75F);
    TEST_ASSERT_EQUAL_FLOAT(0.75F, throttle);
    throttle = cockpit.mapThrottle(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, throttle);

    rates_t rates = cockpit.getRates();
    rates.throttleLimitPercent = 80;
    cockpit.setRates(rates);

    throttle = cockpit.mapThrottle(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    throttle = cockpit.mapThrottle(0.25F);
    TEST_ASSERT_EQUAL_FLOAT(0.2F, throttle);
    throttle = cockpit.mapThrottle(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    throttle = cockpit.mapThrottle(0.75F);
    TEST_ASSERT_EQUAL_FLOAT(0.6F, throttle);
    throttle = cockpit.mapThrottle(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.8F, throttle);

    rates = cockpit.getRates();
    rates.throttleExpo = 255;
    rates.throttleLimitPercent = 100;
    cockpit.setRates(rates);

    throttle = cockpit.mapThrottle(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    throttle = cockpit.mapThrottle(0.25F);
    TEST_ASSERT_EQUAL_FLOAT(0.0625F, throttle);
    throttle = cockpit.mapThrottle(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.25F, throttle);
    throttle = cockpit.mapThrottle(0.75F);
    TEST_ASSERT_EQUAL_FLOAT(0.5625F, throttle);
    throttle = cockpit.mapThrottle(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, throttle);
}

void test_rc_modes()
{
    static RcModes rcModes;
    enum { AUXILIARY_CHANNEL_HORIZON = 4 };
    enum { AUXILIARY_CHANNEL_GPS_RESCUE = 5 };
    enum { BOX_HORIZON_PERMANENT = 2 };
    enum { BOX_GPS_RESCUE_PERMANENT = 46 };

    {
    const uint8_t macIndex = 0;
    rc_modes_activation_condition_t mac = rcModes.get_mode_activation_condition(macIndex);
    const MspBox::box_t* box = MspBox::findBoxByPermanentId(BOX_HORIZON_PERMANENT);
    TEST_ASSERT_FALSE(box == nullptr);

    mac.mode_id = box->id;
    TEST_ASSERT_EQUAL(MspBox::BOX_HORIZON, mac.mode_id);
    mac.auxiliary_channel_index = AUXILIARY_CHANNEL_HORIZON;
    mac.range_start = RcModes::channel_value_to_step(1250);
    mac.range_end = RcModes::channel_value_to_step(1450);

    rcModes.set_mode_activation_condition(macIndex, mac);
    rcModes.analyze_mode_activation_conditions();
    }

    {
    const uint8_t macIndex = 1;
    rc_modes_activation_condition_t mac = rcModes.get_mode_activation_condition(macIndex);
    const MspBox::box_t* box = MspBox::findBoxByPermanentId(BOX_GPS_RESCUE_PERMANENT);
    TEST_ASSERT_FALSE(box == nullptr);

    mac.mode_id = box->id;
    TEST_ASSERT_EQUAL(MspBox::BOX_GPS_RESCUE, mac.mode_id);
    mac.auxiliary_channel_index = AUXILIARY_CHANNEL_GPS_RESCUE;
    mac.range_start = RcModes::channel_value_to_step(1750);
    mac.range_end = RcModes::channel_value_to_step(1850);

    rcModes.set_mode_activation_condition(macIndex, mac);
    rcModes.analyze_mode_activation_conditions();
    }

    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_HORIZON, 1100);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.is_mode_active(MspBox::BOX_HORIZON));

    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_HORIZON, 1300);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.is_mode_active(MspBox::BOX_HORIZON));

    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_GPS_RESCUE, 1500);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.is_mode_active(MspBox::BOX_GPS_RESCUE));

    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_GPS_RESCUE, 1800);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.is_mode_active(MspBox::BOX_GPS_RESCUE));
}

void test_rc_modes_init()
{
    static RcModes rcModes;
    // cppcheck-suppress duplicateExpression
    enum { AUXILIARY_CHANNEL_ARM = ReceiverBase::AUX1 - ReceiverBase::AUX1 }; // NOLINT(misc-redundant-expression)
    enum { AUXILIARY_CHANNEL_ANGLE_MODE = ReceiverBase::AUX2  - ReceiverBase::AUX1 };
    enum { AUXILIARY_CHANNEL_ALTITUDE_HOLD = ReceiverBase::AUX3  - ReceiverBase::AUX1 };
    enum { BOX_ARM_PERMANENT = 0 };
    enum { BOX_ANGLE_PERMANENT = 1 };
    enum { BOX_ALTHOLD_PERMANENT = 3 };

    static constexpr uint8_t MAC_INDEX_ARM = 0;
    static constexpr uint8_t MAC_INDEX_ANGLE = 1;
    static constexpr uint8_t MAC_INDEX_ALTHOLD = 2;

    rc_modes_activation_condition_t macArm = rcModes.get_mode_activation_condition(MAC_INDEX_ARM);
    const MspBox::box_t* boxArm = MspBox::findBoxByPermanentId(BOX_ARM_PERMANENT);
    TEST_ASSERT_FALSE(boxArm == nullptr);
    macArm.mode_id = boxArm->id;
    TEST_ASSERT_EQUAL(MspBox::BOX_ARM, macArm.mode_id);
    macArm.auxiliary_channel_index = AUXILIARY_CHANNEL_ARM;
    macArm.range_start = ReceiverBase::RANGE_STEP_MID;
    macArm.range_end = ReceiverBase::RANGE_STEP_MAX;
    rcModes.set_mode_activation_condition(MAC_INDEX_ARM, macArm);

    rc_modes_activation_condition_t macAngle = rcModes.get_mode_activation_condition(MAC_INDEX_ANGLE);
    const MspBox::box_t* boxAngle = MspBox::findBoxByPermanentId(BOX_ANGLE_PERMANENT);
    TEST_ASSERT_FALSE(boxAngle == nullptr);
    macAngle.mode_id = boxAngle->id;
    TEST_ASSERT_EQUAL(MspBox::BOX_ANGLE, macAngle.mode_id);
    macAngle.auxiliary_channel_index = AUXILIARY_CHANNEL_ANGLE_MODE;
    macAngle.range_start = ReceiverBase::RANGE_STEP_MID;
    macAngle.range_end = ReceiverBase::RANGE_STEP_MAX;
    rcModes.set_mode_activation_condition(MAC_INDEX_ANGLE, macAngle);

    rc_modes_activation_condition_t macAltitudeHold = rcModes.get_mode_activation_condition(MAC_INDEX_ALTHOLD);
    const MspBox::box_t* boxAltitudeHold = MspBox::findBoxByPermanentId(BOX_ALTHOLD_PERMANENT);
    TEST_ASSERT_FALSE(boxAltitudeHold == nullptr);
    macAltitudeHold.mode_id = boxAltitudeHold->id;
    TEST_ASSERT_EQUAL(MspBox::BOX_ALTITUDE_HOLD, macAltitudeHold.mode_id);
    macAltitudeHold.auxiliary_channel_index = AUXILIARY_CHANNEL_ALTITUDE_HOLD;
    macAltitudeHold.range_start = ReceiverBase::RANGE_STEP_MID;
    macAltitudeHold.range_end = ReceiverBase::RANGE_STEP_MAX;
    rcModes.set_mode_activation_condition(MAC_INDEX_ALTHOLD, macAltitudeHold);

    rcModes.analyze_mode_activation_conditions();

    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ARM, 899);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.is_mode_active(MspBox::BOX_ARM));
    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ARM, 900);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.is_mode_active(MspBox::BOX_ARM));
    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ARM, 1000);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.is_mode_active(MspBox::BOX_ARM));
    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ARM, 1499);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.is_mode_active(MspBox::BOX_ARM));
    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ARM, 1500);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.is_mode_active(MspBox::BOX_ARM));
    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ARM, 2000);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.is_mode_active(MspBox::BOX_ARM));
    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ARM, 2099);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.is_mode_active(MspBox::BOX_ARM));
    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ARM, 2100);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.is_mode_active(MspBox::BOX_ARM));

    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ANGLE_MODE, 1100);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.is_mode_active(MspBox::BOX_ANGLE));
    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ANGLE_MODE, 1600);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.is_mode_active(MspBox::BOX_ANGLE));

    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ALTITUDE_HOLD, 1400);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.is_mode_active(MspBox::BOX_ALTITUDE_HOLD));
    receiver.set_auxiliary_channel_pwm(AUXILIARY_CHANNEL_ALTITUDE_HOLD, 1800);
    rcModes.update_activated_modes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.is_mode_active(MspBox::BOX_ALTITUDE_HOLD));
}

void test_rc_adjustments()
{
    static RC_Adjustments rcAdjustments(&DEFAULTS::RC_AdjustmentConfigs);

    rates_t rates = cockpitRates;
    TEST_ASSERT_EQUAL(7, rates.rcRates[rates_t::ROLL]);

    rcAdjustments.applyStepAdjustment(flightController, blackbox, rates, ADJUSTMENT_ROLL_RC_RATE, 1);
    TEST_ASSERT_EQUAL(8, rates.rcRates[rates_t::ROLL]);

    rcAdjustments.applyAbsoluteAdjustment(flightController, blackbox, rates, ADJUSTMENT_ROLL_RC_RATE, 3);
    TEST_ASSERT_EQUAL(3, rates.rcRates[rates_t::ROLL]);

    static Cockpit cockpit(autopilot, nullptr);
    //TEST_ASSERT_EQUAL(0, cockpit.get_current_rate_profile_index());
    //rcAdjustments.applySelectAdjustment(flightController, cockpit, nullptr, nullptr, ADJUSTMENT_RATE_PROFILE, 1);
    //TEST_ASSERT_EQUAL(1, cockpit.get_current_rate_profile_index());
}

void test_flightmode_flags()
{
    static Cockpit cockpit(autopilot, nullptr);

    TEST_ASSERT_EQUAL(0, cockpit.getFlightModeFlags());
    cockpit.setFlightModeFlag(Cockpit::LOG2_HORIZON_MODE);
    uint32_t flags = Cockpit::HORIZON_MODE;
    TEST_ASSERT_EQUAL(flags, cockpit.getFlightModeFlags());

    cockpit.setFlightModeFlag(Cockpit::LOG2_ANGLE_MODE);
    flags |= Cockpit::ANGLE_MODE;
    TEST_ASSERT_EQUAL(flags, cockpit.getFlightModeFlags());

    cockpit.setFlightModeFlag(Cockpit::LOG2_GPS_RESCUE_MODE);
    flags |= Cockpit::GPS_RESCUE_MODE;
    TEST_ASSERT_EQUAL(flags, cockpit.getFlightModeFlags());
}
// NOLINTEND(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_cockpit);
    RUN_TEST(test_cockpit_passthrough);
    RUN_TEST(test_cockpit_set_passthrough);
    RUN_TEST(test_cockpit_defaults);
    RUN_TEST(test_cockpit_constrain);
    RUN_TEST(test_cockpit_throttle);
    RUN_TEST(test_rc_modes);
    RUN_TEST(test_rc_modes_init);
    RUN_TEST(test_rc_adjustments);
    RUN_TEST(test_flightmode_flags);

    UNITY_END();
}
