#include <Debug.h>
#include <DynamicIdleController.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)
void test_dynamic_idle_controller()
{
    enum { TASK_INTERVAL_MICROSECONDS = 1000 };
    const DynamicIdleController::config_t dynamicIdleControllerConfig = {
        .dyn_idle_min_rpm_100 = 0,
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 50,
        .dyn_idle_d_gain = 50,
        .dyn_idle_max_increase = 150,
    };
    static Debug debug;
    static DynamicIdleController dynamicIdleController(TASK_INTERVAL_MICROSECONDS, debug);
    dynamicIdleController.setConfig(dynamicIdleControllerConfig);
    const float deltaT = static_cast<float>(TASK_INTERVAL_MICROSECONDS) * 0.000001F;

    TEST_ASSERT_EQUAL(0, dynamicIdleController.getConfig().dyn_idle_min_rpm_100);

    TEST_ASSERT_EQUAL_FLOAT(0.0F, dynamicIdleController.calculateSpeedIncrease(0.0F, deltaT));
    const float slowestMotorHz = 1000.0F/ 60.0F; // 1000 RPM
    TEST_ASSERT_EQUAL_FLOAT(0.0F, dynamicIdleController.calculateSpeedIncrease(slowestMotorHz, deltaT));
}

void test_dynamic_idle_controller_p_only()
{
    const float motorHz500RPM = 500.0F/ 60.0F;
    const float motorHz750RPM = 750.0F/ 60.0F;
    const float motorHz1000RPM = 1000.0F/ 60.0F;
    const float motorHz2000RPM = 2000.0F/ 60.0F;

    enum { TASK_INTERVAL_MICROSECONDS = 1000 };
    const DynamicIdleController::config_t dynamicIdleControllerConfig = {
        .dyn_idle_min_rpm_100 = 10, // 10*100 = 1000 rpm
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 0,
        .dyn_idle_d_gain = 0,
        .dyn_idle_max_increase = 150,
    };
    static Debug debug;
    static DynamicIdleController dynamicIdleController(TASK_INTERVAL_MICROSECONDS, debug);
    dynamicIdleController.setConfig(dynamicIdleControllerConfig);
    const float deltaT = static_cast<float>(TASK_INTERVAL_MICROSECONDS) * 0.000001F;

    TEST_ASSERT_EQUAL(10, dynamicIdleController.getConfig().dyn_idle_min_rpm_100);
    TEST_ASSERT_EQUAL_FLOAT(motorHz1000RPM, dynamicIdleController.getMinimumAllowedMotorHz());

    // slowest motor faster than 1000 RPM, so no speed increase
    TEST_ASSERT_EQUAL_FLOAT(0.0F, dynamicIdleController.calculateSpeedIncrease(motorHz2000RPM, deltaT));
    TEST_ASSERT_EQUAL_FLOAT(0.0F, dynamicIdleController.calculateSpeedIncrease(motorHz1000RPM, deltaT));

    // slowest motor slower than 1000 RPM, speed increase
    TEST_ASSERT_EQUAL_FLOAT(0.0625F, dynamicIdleController.calculateSpeedIncrease(motorHz500RPM, deltaT));
    TEST_ASSERT_EQUAL_FLOAT(0.0625F, dynamicIdleController.calculateSpeedIncrease(motorHz500RPM, deltaT));
    // half the speed difference from 1000, so half the output, since PID is P-Term only
    TEST_ASSERT_EQUAL_FLOAT(0.03125F, dynamicIdleController.calculateSpeedIncrease(motorHz750RPM, deltaT));
    TEST_ASSERT_EQUAL_FLOAT(0.03125F, dynamicIdleController.calculateSpeedIncrease(motorHz750RPM, deltaT));
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_dynamic_idle_controller);
    RUN_TEST(test_dynamic_idle_controller_p_only);

    UNITY_END();
}
