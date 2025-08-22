#include <FastMath.h>
#include <IMU_Filters.h> // test code won't build if this not included
#include <cmath>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_sin_order5()
{
    constexpr float degreesToRadians { static_cast<float>(M_PI) / 180.0F };

    TEST_ASSERT_EQUAL_FLOAT(0.0F, FastMath::sinOrder5(0.0F));
    //TEST_ASSERT_EQUAL_FLOAT(sinf(10.0F*degreesToRadians), FastMath::sinOrder5(10.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000036F, sinf(10.0F*degreesToRadians), FastMath::sinOrder5(10.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, sinf(20.0F*degreesToRadians), FastMath::sinOrder5(20.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, sinf(30.0F*degreesToRadians), FastMath::sinOrder5(30.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000017F, sinf(40.0F*degreesToRadians), FastMath::sinOrder5(40.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000051F, sinf(50.0F*degreesToRadians), FastMath::sinOrder5(50.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000050F, sinf(60.0F*degreesToRadians), FastMath::sinOrder5(60.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000040F, sinf(70.0F*degreesToRadians), FastMath::sinOrder5(70.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000064F, sinf(80.0F*degreesToRadians), FastMath::sinOrder5(80.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000162F, 1.0F, FastMath::sinOrder5(90.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000064F, sinf(100.0F*degreesToRadians), FastMath::sinOrder5(100.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000040F, sinf(110.0F*degreesToRadians), FastMath::sinOrder5(110.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000050F, sinf(120.0F*degreesToRadians), FastMath::sinOrder5(120.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000051F, sinf(130.0F*degreesToRadians), FastMath::sinOrder5(130.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000017F, sinf(140.0F*degreesToRadians), FastMath::sinOrder5(140.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, sinf(150.0F*degreesToRadians), FastMath::sinOrder5(150.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, sinf(160.0F*degreesToRadians), FastMath::sinOrder5(160.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000036F, sinf(170.0F*degreesToRadians), FastMath::sinOrder5(170.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(0.0F,FastMath::sinOrder5(180.0F*degreesToRadians));
}

void test_cos_order5()
{
    constexpr float degreesToRadians { static_cast<float>(M_PI) / 180.0F };

    TEST_ASSERT_FLOAT_WITHIN(0.000162F, 1.0F, FastMath::cosOrder5(0.0F));
    TEST_ASSERT_FLOAT_WITHIN(0.000064F, cosf(10.0F*degreesToRadians), FastMath::cosOrder5(10.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000040F, cosf(20.0F*degreesToRadians), FastMath::cosOrder5(20.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000050F, cosf(30.0F*degreesToRadians), FastMath::cosOrder5(30.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000051F, cosf(40.0F*degreesToRadians), FastMath::cosOrder5(40.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000017F, cosf(50.0F*degreesToRadians), FastMath::cosOrder5(50.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, cosf(60.0F*degreesToRadians), FastMath::cosOrder5(60.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, cosf(70.0F*degreesToRadians), FastMath::cosOrder5(70.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000036F, cosf(80.0F*degreesToRadians), FastMath::cosOrder5(80.0F*degreesToRadians));
    TEST_ASSERT_EQUAL_FLOAT(0.0F,FastMath::cosOrder5(90.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000036F, cosf(100.0F*degreesToRadians), FastMath::cosOrder5(100.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, cosf(110.0F*degreesToRadians), FastMath::cosOrder5(110.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000048F, cosf(120.0F*degreesToRadians), FastMath::cosOrder5(120.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000017F, cosf(130.0F*degreesToRadians), FastMath::cosOrder5(130.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000051F, cosf(140.0F*degreesToRadians), FastMath::cosOrder5(140.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000050F, cosf(150.0F*degreesToRadians), FastMath::cosOrder5(150.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000040F, cosf(160.0F*degreesToRadians), FastMath::cosOrder5(160.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000064F, cosf(170.0F*degreesToRadians), FastMath::cosOrder5(170.0F*degreesToRadians));
    TEST_ASSERT_FLOAT_WITHIN(0.000162F, -1.0F, FastMath::cosOrder5(180.0F*degreesToRadians));
}

void test_sincos()
{
    constexpr float degreesToRadians { static_cast<float>(M_PI) / 180.0F };

    float sin = 0.0F;
    float cos = 0.0F;

    FastMath::sincos(0.0F, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sin);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, cos);

    FastMath::sincos(370.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(370.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(370.0F*degreesToRadians), cos);

    FastMath::sincos(-370.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-370.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-370.0F*degreesToRadians), cos);

    FastMath::sincos(10.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(10.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(10.0F*degreesToRadians), cos);

    FastMath::sincos(20.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(20.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(20.0F*degreesToRadians), cos);

    FastMath::sincos(30.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(30.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(30.0F*degreesToRadians), cos);

    FastMath::sincos(40.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(40.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(40.0F*degreesToRadians), cos);

    FastMath::sincos(50.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(50.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(50.0F*degreesToRadians), cos);

    FastMath::sincos(60.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(60.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(60.0F*degreesToRadians), cos);

    FastMath::sincos(70.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(70.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(70.0F*degreesToRadians), cos);

    FastMath::sincos(80.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(80.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(80.0F*degreesToRadians), cos);

    FastMath::sincos(90.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(90.0F*degreesToRadians), sin);
    TEST_ASSERT_FLOAT_WITHIN(4.4E-08, cosf(90.0F*degreesToRadians), cos);

    FastMath::sincos(100.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(100.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(100.0F*degreesToRadians), cos);

    FastMath::sincos(110.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(110.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(110.0F*degreesToRadians), cos);

    FastMath::sincos(120.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(120.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(120.0F*degreesToRadians), cos);

    FastMath::sincos(130.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(130.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(130.0F*degreesToRadians), cos);

    FastMath::sincos(140.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(140.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(140.0F*degreesToRadians), cos);

    FastMath::sincos(150.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(150.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(150.0F*degreesToRadians), cos);

    FastMath::sincos(160.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(160.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(160.0F*degreesToRadians), cos);

    FastMath::sincos(170.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(170.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(170.0F*degreesToRadians), cos);

    FastMath::sincos(180.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sin);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, cos);

    FastMath::sincos(-10.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-10.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-10.0F*degreesToRadians), cos);

    FastMath::sincos(-20.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-20.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-20.0F*degreesToRadians), cos);

    FastMath::sincos(-30.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-30.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-30.0F*degreesToRadians), cos);

    FastMath::sincos(-40.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-40.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-40.0F*degreesToRadians), cos);

    FastMath::sincos(-50.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-50.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-50.0F*degreesToRadians), cos);

    FastMath::sincos(-60.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-60.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-60.0F*degreesToRadians), cos);

    FastMath::sincos(-70.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-70.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-70.0F*degreesToRadians), cos);

    FastMath::sincos(-80.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-80.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-80.0F*degreesToRadians), cos);

    FastMath::sincos(-90.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-90.0F*degreesToRadians), sin);
    TEST_ASSERT_FLOAT_WITHIN(4.4E-08, cosf(-90.0F*degreesToRadians), cos);

    FastMath::sincos(-100.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-100.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-100.0F*degreesToRadians), cos);

    FastMath::sincos(-110.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-110.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-110.0F*degreesToRadians), cos);

    FastMath::sincos(-120.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-120.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-120.0F*degreesToRadians), cos);

    FastMath::sincos(-130.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-130.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-130.0F*degreesToRadians), cos);

    FastMath::sincos(-140.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-140.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-140.0F*degreesToRadians), cos);

    FastMath::sincos(-150.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-150.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-150.0F*degreesToRadians), cos);

    FastMath::sincos(-160.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-160.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-160.0F*degreesToRadians), cos);

    FastMath::sincos(-170.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(sinf(-170.0F*degreesToRadians), sin);
    TEST_ASSERT_EQUAL_FLOAT(cosf(-170.0F*degreesToRadians), cos);

    FastMath::sincos(180.0F*degreesToRadians, sin, cos);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, sin);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, cos);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_sin_order5);
    RUN_TEST(test_cos_order5);
    RUN_TEST(test_sincos);

    UNITY_END();
}
