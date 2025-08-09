#include <DShotCodec.h>
#include <IMU_Filters.h> // test code won't build if this not included
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)
void test_dshot_quintets()
{
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[0]] == 0);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[1]] == 1);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[2]] == 2);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[3]] == 3);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[4]] == 4);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[5]] == 5);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[6]] == 6);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[7]] == 7);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[8]] == 8);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[9]] == 9);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[10]] == 10);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[11]] == 11);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[12]] == 12);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[13]] == 13);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[14]] == 14);
    static_assert(DShotCodec::quintetToNibble[DShotCodec::nibbleToQuintet[15]] == 15);
}

void test_dshot_codec_mappings()
{
    TEST_ASSERT_EQUAL(0b11010100101111010110, DShotCodec::eRPM_to_GCR20(0b1000001011000110));
    TEST_ASSERT_EQUAL(0b1000001011000110, DShotCodec::GCR20_to_eRPM(0b11010100101111010110));

    TEST_ASSERT_EQUAL(0b010101010101010101010, DShotCodec::GR21_to_GCR20(0b011001100110011001100));
    TEST_ASSERT_EQUAL(0b011001100110011001100, DShotCodec::GR20_to_GCR21(0b10101010101010101010));
}

void test_dshot_codec()
{
    TEST_ASSERT_EQUAL(47, DShotCodec::pwmToDShot(1000));
    TEST_ASSERT_EQUAL(2047, DShotCodec::pwmToDShot(2000));

    TEST_ASSERT_EQUAL(0, DShotCodec::dShotConvert(0));
    TEST_ASSERT_EQUAL(0, DShotCodec::dShotConvert(10));
    TEST_ASSERT_EQUAL(0, DShotCodec::dShotConvert(999));

    TEST_ASSERT_EQUAL(0, DShotCodec::dShotConvert(1000)); // should this be 0 or 48 ?
    //TEST_ASSERT_EQUAL(48, DShotCodec::dShotConvert(1000)); // should this be 0 or 48 ?
    TEST_ASSERT_EQUAL(49, DShotCodec::dShotConvert(1001));
    TEST_ASSERT_EQUAL(51, DShotCodec::dShotConvert(1002));
    TEST_ASSERT_EQUAL(53, DShotCodec::dShotConvert(1003));
    TEST_ASSERT_EQUAL(1047, DShotCodec::dShotConvert(1500));
    TEST_ASSERT_EQUAL(2045, DShotCodec::dShotConvert(1999));
    TEST_ASSERT_EQUAL(2047, DShotCodec::dShotConvert(2000));
    TEST_ASSERT_EQUAL(2047, DShotCodec::dShotConvert(2001));
    TEST_ASSERT_EQUAL(2047, DShotCodec::dShotConvert(2002));
    TEST_ASSERT_EQUAL(2047, DShotCodec::dShotConvert(4000));


    TEST_ASSERT_EQUAL(1542, DShotCodec::dShotShiftAndAddChecksum(48)); //0x606
    TEST_ASSERT_EQUAL(1572, DShotCodec::dShotShiftAndAddChecksum(49)); // 0x624
    TEST_ASSERT_EQUAL(33547, DShotCodec::dShotShiftAndAddChecksum(1048)); // 0x830B
    TEST_ASSERT_EQUAL(65484, DShotCodec::dShotShiftAndAddChecksum(2046)); // 0xFFCC
    TEST_ASSERT_EQUAL(65518, DShotCodec::dShotShiftAndAddChecksum(2047)); // 0xFFEB, 0xFFFF=65535

    // testing out of range values
    TEST_ASSERT_EQUAL(0, DShotCodec::dShotShiftAndAddChecksum(0));
    TEST_ASSERT_EQUAL(34, DShotCodec::dShotShiftAndAddChecksum(1));
    TEST_ASSERT_EQUAL(68, DShotCodec::dShotShiftAndAddChecksum(2));
    TEST_ASSERT_EQUAL(325, DShotCodec::dShotShiftAndAddChecksum(10));

    //TEST_ASSERT_EQUAL(1, DShotCodec::dShotShiftAndAddChecksum(2048));
    //TEST_ASSERT_EQUAL(35, DShotCodec::dShotShiftAndAddChecksum(2049));
    //TEST_ASSERT_EQUAL(69, DShotCodec::dShotShiftAndAddChecksum(2050));
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

// See for example for testing GCR encoding https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625
// see also https://elmagnifico.tech/2023/04/07/bi-directional-DSHOT/

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_dshot_codec_mappings);
    RUN_TEST(test_dshot_codec);

    UNITY_END();
}
