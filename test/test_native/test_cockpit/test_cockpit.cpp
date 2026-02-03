#include "Autopilot.h"
#include "Cockpit.h"
#include "Defaults.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "NonVolatileStorage.h"

#include <AHRS.h>
#include <AHRS_MessageQueue.h>
#include <Debug.h>
#include <IMU_Null.h>
#include <MotorMixerBase.h>
#include <ReceiverVirtual.h>
#include <SensorFusion.h>


#include <unity.h>

// NOLINTBEGIN(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif

static MadgwickFilter sensorFusionFilter;
static IMU_Null imu(IMU_Base::XPOS_YPOS_ZPOS);
enum { MOTOR_COUNT = 4, SERVO_COUNT = 0 };
static Debug debug;
static IMU_Filters imuFilters(MOTOR_COUNT, debug, 0.0F);
static MotorMixerBase motorMixer(MotorMixerBase::QUAD_X, MOTOR_COUNT, SERVO_COUNT, &debug);
static AHRS_MessageQueue ahrsMessageQueue;
static FlightController flightController(AHRS_TASK_INTERVAL_MICROSECONDS, 1, motorMixer, ahrsMessageQueue, debug);
static AHRS ahrs(AHRS::TIMER_DRIVEN, flightController, sensorFusionFilter, imu, imuFilters);
static Autopilot autopilot(ahrsMessageQueue);
static NonVolatileStorage nvs;
static ReceiverVirtual receiver;


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
    static Cockpit cockpit(flightController, autopilot, imuFilters, debug, nvs, nullptr);
    cockpit.setRates(cockpitRates, flightController);

    rates_t rates = cockpit.getRates();

    // rates.rcRates apply a linear scale factor
    rates.rcRates = {100, 100, 100};
    rates.rcExpos = {0, 0, 0};
    rates.rates = {0, 0, 0};
    //rates.ratesType = rates_t::RATES_TYPE_ACTUAL;
    cockpit.setRates(rates, flightController);

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
    cockpit.setRates(rates, flightController);

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
    static Cockpit cockpit(flightController, autopilot, imuFilters, debug, nvs, nullptr);

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
    static Cockpit cockpit(flightController, autopilot, imuFilters, debug, nvs, nullptr);

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
    static Cockpit cockpit(flightController, autopilot, imuFilters, debug, nvs, nullptr);
    cockpit.setRates(cockpitRates, flightController);

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
    static Cockpit cockpit(flightController, autopilot, imuFilters, debug, nvs, nullptr);
    cockpit.setRates(cockpitRates, flightController);

    rates_t rates = cockpit.getRates(); // NOLINT(misc-const-correctness)
    rates.rcRates = {200, 200, 200};
    cockpit.setRates(rates, flightController);

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
    static Cockpit cockpit(flightController, autopilot, imuFilters, debug, nvs, nullptr);
    cockpit.setRates(cockpitRates, flightController);

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
    cockpit.setRates(rates, flightController);

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
    cockpit.setRates(rates, flightController);

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
    static RC_Modes rcModes;
    enum { AUXILIARY_CHANNEL_HORIZON = 4 };
    enum { AUXILIARY_CHANNEL_GPS_RESCUE = 5 };
    enum { BOX_HORIZON_PERMANENT = 2 };
    enum { BOX_GPS_RESCUE_PERMANENT = 46 };

    {
    const uint8_t macIndex = 0;
    RC_Modes::mode_activation_condition_t mac = rcModes.getModeActivationCondition(macIndex);
    const MSP_Box::box_t* box = MSP_Box::findBoxByPermanentId(BOX_HORIZON_PERMANENT);
    TEST_ASSERT_FALSE(box == nullptr);

    mac.modeId = static_cast<MSP_Box::id_e>(box->id);
    TEST_ASSERT_EQUAL(MSP_Box::BOX_HORIZON, mac.modeId);
    mac.auxiliaryChannelIndex = AUXILIARY_CHANNEL_HORIZON;
    mac.range.startStep = RC_Modes::channelValueToStep(1250);
    mac.range.endStep = RC_Modes::channelValueToStep(1450);

    rcModes.setModeActivationCondition(macIndex, mac);
    rcModes.analyzeModeActivationConditions();
    }

    {
    const uint8_t macIndex = 1;
    RC_Modes::mode_activation_condition_t mac = rcModes.getModeActivationCondition(macIndex);
    const MSP_Box::box_t* box = MSP_Box::findBoxByPermanentId(BOX_GPS_RESCUE_PERMANENT);
    TEST_ASSERT_FALSE(box == nullptr);

    mac.modeId = static_cast<MSP_Box::id_e>(box->id);
    TEST_ASSERT_EQUAL(MSP_Box::BOX_GPS_RESCUE, mac.modeId);
    mac.auxiliaryChannelIndex = AUXILIARY_CHANNEL_GPS_RESCUE;
    mac.range.startStep = RC_Modes::channelValueToStep(1750);
    mac.range.endStep = RC_Modes::channelValueToStep(1850);

    rcModes.setModeActivationCondition(macIndex, mac);
    rcModes.analyzeModeActivationConditions();
    }

    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_HORIZON, 1100);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.isModeActive(MSP_Box::BOX_HORIZON));

    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_HORIZON, 1300);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.isModeActive(MSP_Box::BOX_HORIZON));

    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_GPS_RESCUE, 1500);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.isModeActive(MSP_Box::BOX_GPS_RESCUE));

    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_GPS_RESCUE, 1800);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.isModeActive(MSP_Box::BOX_GPS_RESCUE));
}

void test_rc_modes_init()
{
    static RC_Modes rcModes;
    enum { AUXILIARY_CHANNEL_ARM = ReceiverBase::AUX1 - ReceiverBase::AUX1 }; // NOLINT(misc-redundant-expression)
    enum { AUXILIARY_CHANNEL_ANGLE_MODE = ReceiverBase::AUX2  - ReceiverBase::AUX1 };
    enum { AUXILIARY_CHANNEL_ALTITUDE_HOLD = ReceiverBase::AUX3  - ReceiverBase::AUX1 };
    enum { BOX_ARM_PERMANENT = 0 };
    enum { BOX_ANGLE_PERMANENT = 1 };
    enum { BOX_ALTHOLD_PERMANENT = 3 };

    static constexpr uint8_t MAC_INDEX_ARM = 0;
    static constexpr uint8_t MAC_INDEX_ANGLE = 1;
    static constexpr uint8_t MAC_INDEX_ALTHOLD = 2;

    RC_Modes::mode_activation_condition_t macArm = rcModes.getModeActivationCondition(MAC_INDEX_ARM);
    const MSP_Box::box_t* boxArm = MSP_Box::findBoxByPermanentId(BOX_ARM_PERMANENT);
    TEST_ASSERT_FALSE(boxArm == nullptr);
    macArm.modeId = static_cast<MSP_Box::id_e>(boxArm->id);
    TEST_ASSERT_EQUAL(MSP_Box::BOX_ARM, macArm.modeId);
    macArm.auxiliaryChannelIndex = AUXILIARY_CHANNEL_ARM;
    macArm.range.startStep = ReceiverBase::RANGE_STEP_MID;
    macArm.range.endStep = ReceiverBase::RANGE_STEP_MAX;
    rcModes.setModeActivationCondition(MAC_INDEX_ARM, macArm);

    RC_Modes::mode_activation_condition_t macAngle = rcModes.getModeActivationCondition(MAC_INDEX_ANGLE);
    const MSP_Box::box_t* boxAngle = MSP_Box::findBoxByPermanentId(BOX_ANGLE_PERMANENT);
    TEST_ASSERT_FALSE(boxAngle == nullptr);
    macAngle.modeId = static_cast<MSP_Box::id_e>(boxAngle->id);
    TEST_ASSERT_EQUAL(MSP_Box::BOX_ANGLE, macAngle.modeId);
    macAngle.auxiliaryChannelIndex = AUXILIARY_CHANNEL_ANGLE_MODE;
    macAngle.range.startStep = ReceiverBase::RANGE_STEP_MID;
    macAngle.range.endStep = ReceiverBase::RANGE_STEP_MAX;
    rcModes.setModeActivationCondition(MAC_INDEX_ANGLE, macAngle);

    RC_Modes::mode_activation_condition_t macAltitudeHold = rcModes.getModeActivationCondition(MAC_INDEX_ALTHOLD);
    const MSP_Box::box_t* boxAltitudeHold = MSP_Box::findBoxByPermanentId(BOX_ALTHOLD_PERMANENT);
    TEST_ASSERT_FALSE(boxAltitudeHold == nullptr);
    macAltitudeHold.modeId = static_cast<MSP_Box::id_e>(boxAltitudeHold->id);
    TEST_ASSERT_EQUAL(MSP_Box::BOX_ALTITUDE_HOLD, macAltitudeHold.modeId);
    macAltitudeHold.auxiliaryChannelIndex = AUXILIARY_CHANNEL_ALTITUDE_HOLD;
    macAltitudeHold.range.startStep = ReceiverBase::RANGE_STEP_MID;
    macAltitudeHold.range.endStep = ReceiverBase::RANGE_STEP_MAX;
    rcModes.setModeActivationCondition(MAC_INDEX_ALTHOLD, macAltitudeHold);

    rcModes.analyzeModeActivationConditions();

    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ARM, 899);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.isModeActive(MSP_Box::BOX_ARM));
    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ARM, 900);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.isModeActive(MSP_Box::BOX_ARM));
    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ARM, 1000);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.isModeActive(MSP_Box::BOX_ARM));
    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ARM, 1499);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.isModeActive(MSP_Box::BOX_ARM));
    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ARM, 1500);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.isModeActive(MSP_Box::BOX_ARM));
    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ARM, 2000);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.isModeActive(MSP_Box::BOX_ARM));
    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ARM, 2099);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.isModeActive(MSP_Box::BOX_ARM));
    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ARM, 2100);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.isModeActive(MSP_Box::BOX_ARM));

    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ANGLE_MODE, 1100);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.isModeActive(MSP_Box::BOX_ANGLE));
    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ANGLE_MODE, 1600);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.isModeActive(MSP_Box::BOX_ANGLE));

    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ALTITUDE_HOLD, 1400);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(false, rcModes.isModeActive(MSP_Box::BOX_ALTITUDE_HOLD));
    receiver.setAuxiliaryChannelPWM(AUXILIARY_CHANNEL_ALTITUDE_HOLD, 1800);
    rcModes.updateActivatedModes(receiver);
    TEST_ASSERT_EQUAL(true, rcModes.isModeActive(MSP_Box::BOX_ALTITUDE_HOLD));
}

void test_rc_adjustments()
{
    static RC_Adjustments rcAdjustments(&DEFAULTS::RC_AdjustmentConfigs);

    rates_t rates = cockpitRates;
    TEST_ASSERT_EQUAL(7, rates.rcRates[rates_t::ROLL]);

    rcAdjustments.applyStepAdjustment(flightController, rates, ADJUSTMENT_ROLL_RC_RATE, 1);
    TEST_ASSERT_EQUAL(8, rates.rcRates[rates_t::ROLL]);

    rcAdjustments.applyAbsoluteAdjustment(flightController, rates, ADJUSTMENT_ROLL_RC_RATE, 3);
    TEST_ASSERT_EQUAL(3, rates.rcRates[rates_t::ROLL]);

    static Cockpit cockpit(flightController, autopilot, imuFilters, debug, nvs, nullptr);
    TEST_ASSERT_EQUAL(0, cockpit.getCurrentRateProfileIndex());
    rcAdjustments.applySelectAdjustment(flightController, cockpit, nullptr, ADJUSTMENT_RATE_PROFILE, 1);
    TEST_ASSERT_EQUAL(1, cockpit.getCurrentRateProfileIndex());
}

void test_flightmode_flags()
{
    static Cockpit cockpit(flightController, autopilot, imuFilters, debug, nvs, nullptr);

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

    /*RUN_TEST(test_cockpit);
    RUN_TEST(test_cockpit_passthrough);
    RUN_TEST(test_cockpit_set_passthrough);
    RUN_TEST(test_cockpit_defaults);
    RUN_TEST(test_cockpit_constrain);
    RUN_TEST(test_cockpit_throttle);
    RUN_TEST(test_rc_modes);
    RUN_TEST(test_rc_modes_init);*/
    RUN_TEST(test_rc_adjustments);
    //RUN_TEST(test_flightmode_flags);

    UNITY_END();
}
