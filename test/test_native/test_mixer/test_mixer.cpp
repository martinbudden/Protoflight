#include <IMU_Filters.h> // test code won't build if this not included
#include <Mixers.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)
void test_mixer_roll()
{
    std::array<float, 4> motorOutputs {};
    MotorMixerBase::commands_t commands {};
    float motorOutputMin = 0.0F;
    float undershoot = 0.0F;
    float overshoot = 0.0F;
    float throttle = 0.0F;

    throttle = mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);

    TEST_ASSERT_EQUAL_FLOAT(0.0F, undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motorOutputs[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motorOutputs[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motorOutputs[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motorOutputs[3]);

    commands.throttle = 0.4F;
    commands.roll = 0.3F;
    throttle = mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[0]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[1]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[2]); // throttle + commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[3]); // throttle + commands.roll

    commands.throttle = 0.8F;
    commands.roll = 0.3F;
    throttle = mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[0]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[1]); // throttle - commands.roll
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[2]); // throttle + commands.roll
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[3]); // throttle + commands.roll
}

void test_mixer_pitch()
{
    std::array<float, 4> motorOutputs {};
    MotorMixerBase::commands_t commands {};
    float motorOutputMin = 0.0F;
    float undershoot = 0.0F;
    float overshoot = 0.0F;
    float throttle = 0.0F;

    commands.throttle = 0.4F;
    commands.pitch = 0.3F;
    throttle = mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[0]); // throttle + commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[1]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[2]); // throttle + commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[3]); // throttle - commands.pitch

    commands.throttle = 0.8F;
    commands.pitch = 0.3F;
    throttle = mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[0]); // throttle + commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[1]); // throttle - commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[2]); // throttle + commands.pitch
    TEST_ASSERT_EQUAL_FLOAT(0.4F, motorOutputs[3]); // throttle - commands.pitch
}

void test_mixer_yaw()
{
    std::array<float, 4> motorOutputs {};
    MotorMixerBase::commands_t commands {};
    float motorOutputMin = 0.0F;
    float undershoot = 0.0F;
    float overshoot = 0.0F;
    float throttle = 0.0F;

    commands.throttle = 0.4F;
    commands.yaw = 0.3F;
    throttle = mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.4F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.1F, motorOutputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.7F, motorOutputs[3]); // throttle + commands.yaw

    // this will give an undershoot of -0.1F, so commands.yaw should be adjusted to 0.2F
    commands.throttle = 0.4F;
    commands.yaw = 0.3F;
    motorOutputMin = 0.2F;
    throttle = mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(-0.1F, undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motorOutputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motorOutputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[3]); // throttle + commands.yaw

    // this will give an undershoot of -0.1F, so commands.yaw should be adjusted to -0.2F
    commands.throttle = 0.4F;
    commands.yaw = -0.3F;
    motorOutputMin = 0.2F;
    throttle = mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(-0.1F, undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.5F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motorOutputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.2F, motorOutputs[3]); // throttle + commands.yaw

    // this will give an overshoot of 0.1F, so commands.yaw should be adjusted to 0.2F
    commands.throttle = 0.8F;
    commands.yaw = 0.3F;
    motorOutputMin = 0.0F;
    throttle = mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[3]); // throttle + commands.yaw

    // this will give an overshoot of 0.1F, so commands.yaw should be adjusted to -0.2F
    commands.throttle = 0.8F;
    commands.yaw = -0.3F;
    motorOutputMin = 0.0F;
    throttle = mixQuadX(motorOutputs, commands, motorOutputMin, undershoot, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, undershoot);
    TEST_ASSERT_EQUAL_FLOAT(0.1F, overshoot);
    TEST_ASSERT_EQUAL_FLOAT(0.7F, throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[0]); // throttle + commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[1]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(1.0F, motorOutputs[2]); // throttle - commands.yaw
    TEST_ASSERT_EQUAL_FLOAT(0.6F, motorOutputs[3]); // throttle + commands.yaw
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_mixer_roll);
    RUN_TEST(test_mixer_pitch);
    RUN_TEST(test_mixer_yaw);

    UNITY_END();
}
