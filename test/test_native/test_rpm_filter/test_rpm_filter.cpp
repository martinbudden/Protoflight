#include "RPM_Filter.h"
#include <cmath>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_trigonometry_functions()
{
    constexpr float degreesToRadians { static_cast<float>(M_PI) / 180.0F };

    TEST_ASSERT_EQUAL_FLOAT(0.0F, RPM_Filter::sinOrder5(0.0F));
    //TEST_ASSERT_EQUAL_FLOAT(sinf(10.0F*degreesToRadians), RPM_Filter::sinOrder5(10.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000036F, sinf(10.0F*degreesToRadians), RPM_Filter::sinOrder5(10.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, sinf(20.0F*degreesToRadians), RPM_Filter::sinOrder5(20.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, sinf(30.0F*degreesToRadians), RPM_Filter::sinOrder5(30.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000017F, sinf(40.0F*degreesToRadians), RPM_Filter::sinOrder5(40.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000051F, sinf(50.0F*degreesToRadians), RPM_Filter::sinOrder5(50.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000050F, sinf(60.0F*degreesToRadians), RPM_Filter::sinOrder5(60.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000040F, sinf(70.0F*degreesToRadians), RPM_Filter::sinOrder5(70.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000064F, sinf(80.0F*degreesToRadians), RPM_Filter::sinOrder5(80.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000162F, 1.0F, RPM_Filter::sinOrder5(90.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000064F, sinf(100.0F*degreesToRadians), RPM_Filter::sinOrder5(100.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000040F, sinf(110.0F*degreesToRadians), RPM_Filter::sinOrder5(110.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000050F, sinf(120.0F*degreesToRadians), RPM_Filter::sinOrder5(120.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000051F, sinf(130.0F*degreesToRadians), RPM_Filter::sinOrder5(130.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000017F, sinf(140.0F*degreesToRadians), RPM_Filter::sinOrder5(140.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, sinf(150.0F*degreesToRadians), RPM_Filter::sinOrder5(150.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, sinf(160.0F*degreesToRadians), RPM_Filter::sinOrder5(160.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000036F, sinf(170.0F*degreesToRadians), RPM_Filter::sinOrder5(170.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(0.0F,RPM_Filter::sinOrder5(180.0F*degreesToRadians));

    TEST_ASSERT_FLOAT_WITHIN(0.000162F, 1.0F, RPM_Filter::cosOrder5(0.0F));
    TEST_ASSERT_FLOAT_WITHIN(0.000064F, cosf(10.0F*degreesToRadians), RPM_Filter::cosOrder5(10.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000040F, cosf(20.0F*degreesToRadians), RPM_Filter::cosOrder5(20.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000050F, cosf(30.0F*degreesToRadians), RPM_Filter::cosOrder5(30.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000051F, cosf(40.0F*degreesToRadians), RPM_Filter::cosOrder5(40.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000017F, cosf(50.0F*degreesToRadians), RPM_Filter::cosOrder5(50.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, cosf(60.0F*degreesToRadians), RPM_Filter::cosOrder5(60.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, cosf(70.0F*degreesToRadians), RPM_Filter::cosOrder5(70.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000036F, cosf(80.0F*degreesToRadians), RPM_Filter::cosOrder5(80.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(0.0F,RPM_Filter::cosOrder5(90.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000036F, cosf(100.0F*degreesToRadians), RPM_Filter::cosOrder5(100.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, cosf(110.0F*degreesToRadians), RPM_Filter::cosOrder5(110.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, cosf(120.0F*degreesToRadians), RPM_Filter::cosOrder5(120.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000017F, cosf(130.0F*degreesToRadians), RPM_Filter::cosOrder5(130.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000051F, cosf(140.0F*degreesToRadians), RPM_Filter::cosOrder5(140.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000050F, cosf(150.0F*degreesToRadians), RPM_Filter::cosOrder5(150.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000040F, cosf(160.0F*degreesToRadians), RPM_Filter::cosOrder5(160.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000064F, cosf(170.0F*degreesToRadians), RPM_Filter::cosOrder5(170.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000162F, -1.0F, RPM_Filter::cosOrder5(180.0F*degreesToRadians));
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_trigonometry_functions);

    UNITY_END();
}
