#include "Autopilot.h"
#include "Cockpit.h"
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


static const Cockpit::rates_t cockpitRates {
    .rateLimits = { Cockpit::RATE_LIMIT_MAX, Cockpit::RATE_LIMIT_MAX, Cockpit::RATE_LIMIT_MAX},
    .rcRates = { 7, 7, 7 },
    .rcExpos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttleMidpoint = 50,
    .throttleExpo = 0,
    .throttleLimitType = Cockpit::THROTTLE_LIMIT_TYPE_OFF,
    .throttleLimitPercent = 100,
    //.ratesType = Cockpit::RATES_TYPE_ACTUAL
};

void setUp() {
}

void tearDown() {
}

void test_cockpit()
{
    static Cockpit cockpit(receiver, flightController, autopilot, imuFilters, debug, nvs);
    cockpit.setRates(cockpitRates);

    Cockpit::rates_t rates = cockpit.getRates();

    // rates.rcRates apply a linear scale factor
    rates.rcRates = {100, 100, 100};
    rates.rcExpos = {0, 0, 0};
    rates.rates = {0, 0, 0};
    //rates.ratesType = Cockpit::RATES_TYPE_ACTUAL;
    cockpit.setRates(rates);

    float roll = cockpit.applyRates(Cockpit::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0, roll);

    // rates.rates apply a nonlinear scale factor
    rates.rcRates = {0, 0, 0};
    rates.rates = {60, 60, 60};
    cockpit.setRates(rates);

    roll = cockpit.applyRates(Cockpit::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(37.5F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.50F);
    TEST_ASSERT_EQUAL_FLOAT(150.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(337.5F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(600.0F, roll); // 100.0F / (1.0F - 0.60F)
}

void test_cockpit_passthrough()
{
    static Cockpit cockpit(receiver, flightController, autopilot, imuFilters, debug, nvs);

    float roll = cockpit.applyRates(Cockpit::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
}

void test_cockpit_set_passthrough()
{
    static Cockpit cockpit(receiver, flightController, autopilot, imuFilters, debug, nvs);

    cockpit.setRatesToPassThrough();

    float roll = cockpit.applyRates(Cockpit::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
}

void test_cockpit_defaults()
{
    static Cockpit cockpit(receiver, flightController, autopilot, imuFilters, debug, nvs);
    cockpit.setRates(cockpitRates);

    const Cockpit::rates_t rates = cockpit.getRates();

    TEST_ASSERT_EQUAL(Cockpit::RATE_LIMIT_MAX, rates.rateLimits[0]);
    TEST_ASSERT_EQUAL(Cockpit::RATE_LIMIT_MAX, rates.rateLimits[1]);
    TEST_ASSERT_EQUAL(Cockpit::RATE_LIMIT_MAX, rates.rateLimits[2]);
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
    TEST_ASSERT_EQUAL(Cockpit::THROTTLE_LIMIT_TYPE_OFF, rates.throttleLimitType);
    TEST_ASSERT_EQUAL(100, rates.throttleLimitPercent);
    //TEST_ASSERT_EQUAL(Cockpit::RATES_TYPE_ACTUAL, rates.ratesType);

    float roll = cockpit.applyRates(Cockpit::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(55.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(185.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(390.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(670.0F, roll);
}

void test_cockpit_constrain()
{
    static Cockpit cockpit(receiver, flightController, autopilot, imuFilters, debug, nvs);
    cockpit.setRates(cockpitRates);

    Cockpit::rates_t rates = cockpit.getRates(); // NOLINT(misc-const-correctness)
    rates.rcRates = {200, 200, 200};
    cockpit.setRates(rates);

    float roll = cockpit.applyRates(Cockpit::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(1500.0F, roll);
    roll = cockpit.applyRates(Cockpit::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1998.0F, roll);
}

void test_cockpit_throttle()
{
    static Cockpit cockpit(receiver, flightController, autopilot, imuFilters, debug, nvs);
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

    Cockpit::rates_t rates = cockpit.getRates();
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
    static RC_Modes rcModes;
    enum { AUXILIARY_CHANNEL_HORIZON = 4 };
    enum { AUXILIARY_CHANNEL_GPS_RESCUE = 5 };
    enum { BOX_HORIZON_PERMANENT = 2 };
    enum { BOX_GPS_RESCUE_PERMANENT = 46 };

    auto& modeActivationConditions = rcModes.getModeActivationConditions();
    {
    const uint8_t macIndex = 0;
    RC_Modes::mode_activation_condition_t& mac = modeActivationConditions[macIndex];
    const MSP_Box::msp_box_t* box = MSP_Box::findBoxByPermanentId(BOX_HORIZON_PERMANENT);
    TEST_ASSERT_FALSE(box == nullptr);

    mac.modeId = static_cast<MSP_Box::box_id_e>(box->boxId);
    TEST_ASSERT_EQUAL(MSP_Box::BOX_HORIZON, mac.modeId);
    mac.auxChannelIndex = AUXILIARY_CHANNEL_HORIZON;
    mac.range.startStep = RC_Modes::channelValueToStep(1250);
    mac.range.endStep = RC_Modes::channelValueToStep(1450);
    rcModes.analyzeModeActivationConditions();
    }

    {
    const uint8_t macIndex = 1;
    RC_Modes::mode_activation_condition_t& mac = modeActivationConditions[macIndex];
    const MSP_Box::msp_box_t* box = MSP_Box::findBoxByPermanentId(BOX_GPS_RESCUE_PERMANENT);
    TEST_ASSERT_FALSE(box == nullptr);

    mac.modeId = static_cast<MSP_Box::box_id_e>(box->boxId);
    TEST_ASSERT_EQUAL(MSP_Box::BOX_GPS_RESCUE, mac.modeId);
    mac.auxChannelIndex = AUXILIARY_CHANNEL_GPS_RESCUE;
    mac.range.startStep = RC_Modes::channelValueToStep(1750);
    mac.range.endStep = RC_Modes::channelValueToStep(1850);
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

    UNITY_END();
}
