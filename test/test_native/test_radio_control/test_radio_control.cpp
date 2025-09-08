#include "RadioController.h"
#include <ReceiverNull.h>


#include <unity.h>


static const RadioController::rates_t radioControllerRates {
    .rateLimits = { RadioController::RATE_LIMIT_MAX, RadioController::RATE_LIMIT_MAX, RadioController::RATE_LIMIT_MAX},
    .rcRates = { 7, 7, 7 },
    .rcExpos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttleMidpoint = 50,
    .throttleExpo = 0,
    .throttleLimitType = RadioController::THROTTLE_LIMIT_TYPE_OFF,
    .throttleLimitPercent = 100,
    //.ratesType = RadioController::RATES_TYPE_ACTUAL
};

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(misc-const-correctness)
void test_radio_controller()
{
    static ReceiverNull receiver;
    static RadioController radioController(receiver, radioControllerRates);

    RadioController::rates_t rates = radioController.getRates();

    // rates.rcRates apply a linear scale factor
    rates.rcRates = {100, 100, 100};
    rates.rcExpos = {0, 0, 0};
    rates.rates = {0, 0, 0};
    //rates.ratesType = RadioController::RATES_TYPE_ACTUAL;
    radioController.setRates(rates);

    float roll = radioController.applyRates(RadioController::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0, roll);

    // rates.rates apply a nonlinear scale factor
    rates.rcRates = {0, 0, 0};
    rates.rates = {60, 60, 60};
    radioController.setRates(rates);

    roll = radioController.applyRates(RadioController::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(37.5F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.50F);
    TEST_ASSERT_EQUAL_FLOAT(150.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(337.5F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(600.0F, roll); // 100.0F / (1.0F - 0.60F)
}

void test_radio_controller_passthrough()
{
    static ReceiverNull receiver;
    static RadioController radioController(receiver, radioControllerRates);

    radioController.setRatesToPassThrough();

    float roll = radioController.applyRates(RadioController::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(250.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(750.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
}

void test_radio_controller_defaults()
{
    static ReceiverNull receiver;
    static RadioController radioController(receiver, radioControllerRates);

    const RadioController::rates_t rates = radioController.getRates();

    TEST_ASSERT_EQUAL(RadioController::RATE_LIMIT_MAX, rates.rateLimits[0]);
    TEST_ASSERT_EQUAL(RadioController::RATE_LIMIT_MAX, rates.rateLimits[1]);
    TEST_ASSERT_EQUAL(RadioController::RATE_LIMIT_MAX, rates.rateLimits[2]);
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
    TEST_ASSERT_EQUAL(RadioController::THROTTLE_LIMIT_TYPE_OFF, rates.throttleLimitType);
    TEST_ASSERT_EQUAL(100, rates.throttleLimitPercent);
    //TEST_ASSERT_EQUAL(RadioController::RATES_TYPE_ACTUAL, rates.ratesType);

    float roll = radioController.applyRates(RadioController::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(55.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(185.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(390.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(670.0F, roll);
}

void test_radio_controller_constrain()
{
    static ReceiverNull receiver;
    static RadioController radioController(receiver, radioControllerRates);

    RadioController::rates_t rates = radioController.getRates(); // NOLINT(misc-const-correctness)
    rates.rcRates = {200, 200, 200};
    radioController.setRates(rates);

    float roll = radioController.applyRates(RadioController::ROLL, 0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.25F);
    TEST_ASSERT_EQUAL_FLOAT(500.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.5F);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 0.75F);
    TEST_ASSERT_EQUAL_FLOAT(1500.0F, roll);
    roll = radioController.applyRates(RadioController::ROLL, 1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1998.0F, roll);
}

void test_radio_controller_throttle()
{
    static ReceiverNull receiver;
    static RadioController radioController(receiver, radioControllerRates);

    float throttle = radioController.mapThrottle(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    throttle = radioController.mapThrottle(0.25F);
    TEST_ASSERT_EQUAL_FLOAT(0.25F, throttle);
    throttle = radioController.mapThrottle(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, throttle);
    throttle = radioController.mapThrottle(0.75F);
    TEST_ASSERT_EQUAL_FLOAT(0.75F, throttle);
    throttle = radioController.mapThrottle(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, throttle);

    RadioController::rates_t rates = radioController.getRates();
    rates.throttleLimitPercent = 80;
    radioController.setRates(rates);

    throttle = radioController.mapThrottle(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    throttle = radioController.mapThrottle(0.25F);
    TEST_ASSERT_EQUAL_FLOAT(0.2F, throttle);
    throttle = radioController.mapThrottle(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    throttle = radioController.mapThrottle(0.75F);
    TEST_ASSERT_EQUAL_FLOAT(0.6F, throttle);
    throttle = radioController.mapThrottle(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.8F, throttle);

    rates = radioController.getRates();
    rates.throttleExpo = 255;
    rates.throttleLimitPercent = 100;
    radioController.setRates(rates);

    throttle = radioController.mapThrottle(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    throttle = radioController.mapThrottle(0.25F);
    TEST_ASSERT_EQUAL_FLOAT(0.0625F, throttle);
    throttle = radioController.mapThrottle(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.25F, throttle);
    throttle = radioController.mapThrottle(0.75F);
    TEST_ASSERT_EQUAL_FLOAT(0.5625F, throttle);
    throttle = radioController.mapThrottle(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, throttle);
}
// NOLINTEND(misc-const-correctness)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_radio_controller);
    RUN_TEST(test_radio_controller_passthrough);
    RUN_TEST(test_radio_controller_defaults);
    RUN_TEST(test_radio_controller_constrain);
    RUN_TEST(test_radio_controller_throttle);

    UNITY_END();
}
