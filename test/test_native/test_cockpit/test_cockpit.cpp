#include "autopilot.h"
#include "cockpit.h"
#include "defaults.h"
#include "flight_controller.h"

#include <rc_modes.h>

#include <ahrs.h>
#include <ahrs_message_queue.h>
#include <receiver_virtual.h>
#include <sensor_fusion.h>


#include <unity.h>

// NOLINTBEGIN(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif

static AhrsMessageQueue ahrs_message_queue;
static FlightController flight_controller(AHRS_TASK_INTERVAL_MICROSECONDS);
static Autopilot autopilot(ahrs_message_queue);
static ReceiverVirtual receiver;
static Blackbox* blackbox =  nullptr;


static const rates_t cockpitRates {
    .rate_limits = { rates_t::LIMIT_MAX, rates_t::LIMIT_MAX, rates_t::LIMIT_MAX},
    .rc_rates = { 7, 7, 7 },
    .rc_expos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttle_midpoint = 50,
    .throttle_expo = 0,
    .throttle_limit_type = rates_t::THROTTLE_LIMIT_TYPE_OFF,
    .throttle_limit_percent = 100,
    //.ratesType = rates_t::RATES_TYPE_ACTUAL
};

void setUp() {
}

void tearDown() {
}

void test_cockpit()
{
    static Cockpit cockpit(autopilot, nullptr);
    cockpit.set_rates(cockpitRates);

    rates_t rates = cockpit.get_rates();

    // rates.rc_rates apply a linear scale factor
    rates.rc_rates = {100, 100, 100};
    rates.rc_expos = {0, 0, 0};
    rates.rates = {0, 0, 0};
    //rates.ratesType = rates_t::RATES_TYPE_ACTUAL;
    cockpit.set_rates(rates);

    float roll = cockpit.apply_rates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0, roll);

    // rates.rates apply a nonlinear scale factor
    rates.rc_rates = {0, 0, 0};
    rates.rates = {60, 60, 60};
    cockpit.set_rates(rates);

    roll = cockpit.apply_rates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(37.5F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.50F);
    TEST_ASSERT_EQUAL_FLOAT(150.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(337.5F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(600.0F, roll); // 100.0F / (1.0F - 0.60F)
}

void test_cockpit_passthrough()
{
    static Cockpit cockpit(autopilot, nullptr);

    float roll = cockpit.apply_rates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
}

void test_cockpit_set_passthrough()
{
    static Cockpit cockpit(autopilot, nullptr);

    cockpit.set_rates_to_pass_through();

    float roll = cockpit.apply_rates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
}

void test_cockpit_defaults()
{
    static Cockpit cockpit(autopilot, nullptr);
    cockpit.set_rates(cockpitRates);

    const rates_t rates = cockpit.get_rates();

    TEST_ASSERT_EQUAL(rates_t::LIMIT_MAX, rates.rate_limits[0]);
    TEST_ASSERT_EQUAL(rates_t::LIMIT_MAX, rates.rate_limits[1]);
    TEST_ASSERT_EQUAL(rates_t::LIMIT_MAX, rates.rate_limits[2]);
    TEST_ASSERT_EQUAL(7, rates.rc_rates[0]);
    TEST_ASSERT_EQUAL(7, rates.rc_rates[1]);
    TEST_ASSERT_EQUAL(7, rates.rc_rates[2]);
    TEST_ASSERT_EQUAL(0, rates.rc_expos[0]);
    TEST_ASSERT_EQUAL(0, rates.rc_expos[1]);
    TEST_ASSERT_EQUAL(0, rates.rc_expos[2]);
    TEST_ASSERT_EQUAL(67, rates.rates[0]);
    TEST_ASSERT_EQUAL(67, rates.rates[1]);
    TEST_ASSERT_EQUAL(67, rates.rates[2]);
    TEST_ASSERT_EQUAL(50, rates.throttle_midpoint);
    TEST_ASSERT_EQUAL(0, rates.throttle_expo);
    TEST_ASSERT_EQUAL(rates_t::THROTTLE_LIMIT_TYPE_OFF, rates.throttle_limit_type);
    TEST_ASSERT_EQUAL(100, rates.throttle_limit_percent);
    //TEST_ASSERT_EQUAL(rates_t::RATES_TYPE_ACTUAL, rates.ratesType);

    float roll = cockpit.apply_rates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(55.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(185.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(390.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(670.0F, roll);
}

void test_cockpit_constrain()
{
    static Cockpit cockpit(autopilot, nullptr);
    cockpit.set_rates(cockpitRates);

    rates_t rates = cockpit.get_rates(); // NOLINT(misc-const-correctness)
    rates.rc_rates = {200, 200, 200};
    cockpit.set_rates(rates);

    float roll = cockpit.apply_rates(rates_t::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(1500.0F, roll);
    roll = cockpit.apply_rates(rates_t::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1998.0F, roll);
}

void test_cockpit_throttle()
{
    static Cockpit cockpit(autopilot, nullptr);
    cockpit.set_rates(cockpitRates);

    float throttle = cockpit.map_throttle(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    throttle = cockpit.map_throttle(0.25F);
    TEST_ASSERT_EQUAL_FLOAT(0.25F, throttle);
    throttle = cockpit.map_throttle(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, throttle);
    throttle = cockpit.map_throttle(0.75F);
    TEST_ASSERT_EQUAL_FLOAT(0.75F, throttle);
    throttle = cockpit.map_throttle(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, throttle);

    rates_t rates = cockpit.get_rates();
    rates.throttle_limit_percent = 80;
    cockpit.set_rates(rates);

    throttle = cockpit.map_throttle(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    throttle = cockpit.map_throttle(0.25F);
    TEST_ASSERT_EQUAL_FLOAT(0.2F, throttle);
    throttle = cockpit.map_throttle(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    throttle = cockpit.map_throttle(0.75F);
    TEST_ASSERT_EQUAL_FLOAT(0.6F, throttle);
    throttle = cockpit.map_throttle(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.8F, throttle);

    rates = cockpit.get_rates();
    rates.throttle_expo = 255;
    rates.throttle_limit_percent = 100;
    cockpit.set_rates(rates);

    throttle = cockpit.map_throttle(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    throttle = cockpit.map_throttle(0.25F);
    TEST_ASSERT_EQUAL_FLOAT(0.0625F, throttle);
    throttle = cockpit.map_throttle(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.25F, throttle);
    throttle = cockpit.map_throttle(0.75F);
    TEST_ASSERT_EQUAL_FLOAT(0.5625F, throttle);
    throttle = cockpit.map_throttle(1.0F);
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
    const uint8_t mac_index = 0;
    rc_modes_activation_condition_t mac = rcModes.get_mode_activation_condition(mac_index);
    const MspBox::box_t* box = MspBox::find_box_by_permanent_id(BOX_HORIZON_PERMANENT);
    TEST_ASSERT_FALSE(box == nullptr);

    mac.mode_id = box->id;
    TEST_ASSERT_EQUAL(MspBox::BOX_HORIZON, mac.mode_id);
    mac.auxiliary_channel_index = AUXILIARY_CHANNEL_HORIZON;
    mac.range_start = RcModes::channel_value_to_step(1250);
    mac.range_end = RcModes::channel_value_to_step(1450);

    rcModes.set_mode_activation_condition(mac_index, mac);
    rcModes.analyze_mode_activation_conditions();
    }

    {
    const uint8_t mac_index = 1;
    rc_modes_activation_condition_t mac = rcModes.get_mode_activation_condition(mac_index);
    const MspBox::box_t* box = MspBox::find_box_by_permanent_id(BOX_GPS_RESCUE_PERMANENT);
    TEST_ASSERT_FALSE(box == nullptr);

    mac.mode_id = box->id;
    TEST_ASSERT_EQUAL(MspBox::BOX_GPS_RESCUE, mac.mode_id);
    mac.auxiliary_channel_index = AUXILIARY_CHANNEL_GPS_RESCUE;
    mac.range_start = RcModes::channel_value_to_step(1750);
    mac.range_end = RcModes::channel_value_to_step(1850);

    rcModes.set_mode_activation_condition(mac_index, mac);
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
    const MspBox::box_t* boxArm = MspBox::find_box_by_permanent_id(BOX_ARM_PERMANENT);
    TEST_ASSERT_FALSE(boxArm == nullptr);
    macArm.mode_id = boxArm->id;
    TEST_ASSERT_EQUAL(MspBox::BOX_ARM, macArm.mode_id);
    macArm.auxiliary_channel_index = AUXILIARY_CHANNEL_ARM;
    macArm.range_start = ReceiverBase::RANGE_STEP_MID;
    macArm.range_end = ReceiverBase::RANGE_STEP_MAX;
    rcModes.set_mode_activation_condition(MAC_INDEX_ARM, macArm);

    rc_modes_activation_condition_t macAngle = rcModes.get_mode_activation_condition(MAC_INDEX_ANGLE);
    const MspBox::box_t* boxAngle = MspBox::find_box_by_permanent_id(BOX_ANGLE_PERMANENT);
    TEST_ASSERT_FALSE(boxAngle == nullptr);
    macAngle.mode_id = boxAngle->id;
    TEST_ASSERT_EQUAL(MspBox::BOX_ANGLE, macAngle.mode_id);
    macAngle.auxiliary_channel_index = AUXILIARY_CHANNEL_ANGLE_MODE;
    macAngle.range_start = ReceiverBase::RANGE_STEP_MID;
    macAngle.range_end = ReceiverBase::RANGE_STEP_MAX;
    rcModes.set_mode_activation_condition(MAC_INDEX_ANGLE, macAngle);

    rc_modes_activation_condition_t macAltitudeHold = rcModes.get_mode_activation_condition(MAC_INDEX_ALTHOLD);
    const MspBox::box_t* boxAltitudeHold = MspBox::find_box_by_permanent_id(BOX_ALTHOLD_PERMANENT);
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
    static RcAdjustments rc_adjustments(&DEFAULTS::RC_ADJUSTMENT_CONFIGS);

    rates_t rates = cockpitRates;
    TEST_ASSERT_EQUAL(7, rates.rc_rates[rates_t::ROLL]);

    rc_adjustments.apply_step_adjustment(flight_controller, blackbox, rates, ADJUSTMENT_ROLL_RC_RATE, 1);
    TEST_ASSERT_EQUAL(8, rates.rc_rates[rates_t::ROLL]);

    rc_adjustments.apply_absolute_adjustment(flight_controller, blackbox, rates, ADJUSTMENT_ROLL_RC_RATE, 3);
    TEST_ASSERT_EQUAL(3, rates.rc_rates[rates_t::ROLL]);

    static Cockpit cockpit(autopilot, nullptr);
    //TEST_ASSERT_EQUAL(0, cockpit.get_current_rate_profile_index());
    //rc_adjustments.apply_select_adjustment(flight_controller, cockpit, nullptr, nullptr, ADJUSTMENT_RATE_PROFILE, 1);
    //TEST_ASSERT_EQUAL(1, cockpit.get_current_rate_profile_index());
}

void test_flightmode_flags()
{
    static Cockpit cockpit(autopilot, nullptr);

    TEST_ASSERT_EQUAL(0, cockpit.get_flight_mode_flags());
    cockpit.set_flight_mode_flag(Cockpit::LOG2_HORIZON_MODE);
    uint32_t flags = Cockpit::HORIZON_MODE;
    TEST_ASSERT_EQUAL(flags, cockpit.get_flight_mode_flags());

    cockpit.set_flight_mode_flag(Cockpit::LOG2_ANGLE_MODE);
    flags |= Cockpit::ANGLE_MODE;
    TEST_ASSERT_EQUAL(flags, cockpit.get_flight_mode_flags());

    cockpit.set_flight_mode_flag(Cockpit::LOG2_GPS_RESCUE_MODE);
    flags |= Cockpit::GPS_RESCUE_MODE;
    TEST_ASSERT_EQUAL(flags, cockpit.get_flight_mode_flags());
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
