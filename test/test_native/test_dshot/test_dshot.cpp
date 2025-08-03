#include <ESC_DShot.h>
#include <array>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)
void test_dshot_init()
{
    enum { DSHOT_PIN = 4 };

    static ESC_DShot esc150(ESC_DShot::ESC_PROTOCOL_DSHOT150);
    esc150.init(DSHOT_PIN);
    TEST_ASSERT_EQUAL(700, esc150.getDataHighPulseWidth());
    TEST_ASSERT_EQUAL(350, esc150.getDataLowPulseWidth());

    static ESC_DShot esc300(ESC_DShot::ESC_PROTOCOL_DSHOT300);
    esc300.init(DSHOT_PIN);
    TEST_ASSERT_EQUAL(350, esc300.getDataHighPulseWidth());
    TEST_ASSERT_EQUAL(175, esc300.getDataLowPulseWidth());

    static ESC_DShot esc600(ESC_DShot::ESC_PROTOCOL_DSHOT600);
    esc600.init(DSHOT_PIN);
    TEST_ASSERT_EQUAL(175, esc600.getDataHighPulseWidth());
    TEST_ASSERT_EQUAL(87, esc600.getDataLowPulseWidth());
}

void test_dshot()
{
    enum { DSHOT_PIN = 4 };

    static ESC_DShot esc(ESC_DShot::ESC_PROTOCOL_DSHOT300);
    esc.init(DSHOT_PIN);

    TEST_ASSERT_EQUAL(375, esc.nanoSecondsToCycles(2500));
    TEST_ASSERT_EQUAL(750, esc.nanoSecondsToCycles(5000));
    TEST_ASSERT_EQUAL(1125, esc.nanoSecondsToCycles(7500));

    TEST_ASSERT_EQUAL(350, esc.nanoSecondsToCycles(2333));
    TEST_ASSERT_EQUAL(700, esc.nanoSecondsToCycles(4666));
    TEST_ASSERT_EQUAL(1000, esc.nanoSecondsToCycles(6666));

    TEST_ASSERT_EQUAL(47, ESC_DShot::pwmToDShot(1000));
    TEST_ASSERT_EQUAL(2047, ESC_DShot::pwmToDShot(2000));

    TEST_ASSERT_EQUAL(0, ESC_DShot::dShotConvert(0));
    TEST_ASSERT_EQUAL(0, ESC_DShot::dShotConvert(10));
    TEST_ASSERT_EQUAL(0, ESC_DShot::dShotConvert(999));

    TEST_ASSERT_EQUAL(0, ESC_DShot::dShotConvert(1000)); // should this be 0 or 48 ?
    //TEST_ASSERT_EQUAL(48, ESC_DShot::dShotConvert(1000)); // should this be 0 or 48 ?
    TEST_ASSERT_EQUAL(49, ESC_DShot::dShotConvert(1001));
    TEST_ASSERT_EQUAL(51, ESC_DShot::dShotConvert(1002));
    TEST_ASSERT_EQUAL(53, ESC_DShot::dShotConvert(1003));
    TEST_ASSERT_EQUAL(1047, ESC_DShot::dShotConvert(1500));
    TEST_ASSERT_EQUAL(2045, ESC_DShot::dShotConvert(1999));
    TEST_ASSERT_EQUAL(2047, ESC_DShot::dShotConvert(2000));
    TEST_ASSERT_EQUAL(2047, ESC_DShot::dShotConvert(2001));


    TEST_ASSERT_EQUAL(1542, ESC_DShot::dShotShiftAndAddChecksum(48)); //0x606
    TEST_ASSERT_EQUAL(1572, ESC_DShot::dShotShiftAndAddChecksum(49)); // 0x624
    TEST_ASSERT_EQUAL(33547, ESC_DShot::dShotShiftAndAddChecksum(1048)); // 0x830B
    TEST_ASSERT_EQUAL(65484, ESC_DShot::dShotShiftAndAddChecksum(2046)); // 0xFFCC
    TEST_ASSERT_EQUAL(65518, ESC_DShot::dShotShiftAndAddChecksum(2047)); // 0xFFEB, 0xFFFF=65535

    // testing out of range values
    TEST_ASSERT_EQUAL(0, ESC_DShot::dShotShiftAndAddChecksum(0));
    TEST_ASSERT_EQUAL(34, ESC_DShot::dShotShiftAndAddChecksum(1));
    TEST_ASSERT_EQUAL(68, ESC_DShot::dShotShiftAndAddChecksum(2));
    TEST_ASSERT_EQUAL(325, ESC_DShot::dShotShiftAndAddChecksum(10));

    TEST_ASSERT_EQUAL(1, ESC_DShot::dShotShiftAndAddChecksum(2048));
    TEST_ASSERT_EQUAL(35, ESC_DShot::dShotShiftAndAddChecksum(2049));
    TEST_ASSERT_EQUAL(69, ESC_DShot::dShotShiftAndAddChecksum(2050));

    esc.end();
}

void test_dshot_write()
{
    enum { DSHOT_PIN = 4 };

    static ESC_DShot esc(ESC_DShot::ESC_PROTOCOL_DSHOT300);
    esc.init(DSHOT_PIN);

    const uint32_t HI = esc.getDataHighPulseWidth();
    const uint32_t LO = esc.getDataLowPulseWidth();

    esc.write(1000);
    const uint16_t frame1000 = ESC_DShot::dShotShiftAndAddChecksum(ESC_DShot::dShotConvert(1000));
    TEST_ASSERT_EQUAL(0, frame1000);
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(1));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(2));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(3));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(4));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(5));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(6));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(7));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(8));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(9));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(10));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(11));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(12));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(13));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(14));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16)); // check haven't written past end of buffer

    const uint16_t value = ESC_DShot::dShotConvert(1500);
    const uint16_t frame = ESC_DShot::dShotShiftAndAddChecksum(value);
    TEST_ASSERT_EQUAL(33508, frame); // 0x82E4, 1000 0010 1110 0100
    esc.write(1500);
    // 1000 0010 1110 0100
    // 1000
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(1));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(2));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(3));
    // 0010
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(4));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(5));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(6));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(7));
    // 1110
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(8));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(9));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(10));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(11));
    //0100
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(12));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(13));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(14));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16));

    const uint16_t frame2000 = ESC_DShot::dShotShiftAndAddChecksum(ESC_DShot::dShotConvert(2000));
    TEST_ASSERT_EQUAL(65518, frame2000); // 0xFFEE, 1111 1111 1110 1110
    esc.write(2000);
    // 1111
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(1));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(2));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(3));
    // 1110
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(12));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(13));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(14));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16));
}

void test_dshot_write_channel_b()
{
    enum { DSHOT_PIN = 4 };

    static ESC_DShot esc(ESC_DShot::ESC_PROTOCOL_DSHOT300);
    esc.init(DSHOT_PIN);
    esc.setUseHighOrderBits(true);

    // channel B uses high order bits
    const uint32_t HI = esc.getDataHighPulseWidth() << 16;
    const uint32_t LO = esc.getDataLowPulseWidth() << 16;

    esc.write(1000);
    const uint16_t frame1000 = ESC_DShot::dShotShiftAndAddChecksum(ESC_DShot::dShotConvert(1000));
    TEST_ASSERT_EQUAL(0, frame1000);
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16)); // check haven't written past end of buffer

    const uint16_t value = ESC_DShot::dShotConvert(1500);
    const uint16_t frame = ESC_DShot::dShotShiftAndAddChecksum(value);
    TEST_ASSERT_EQUAL(33508, frame); // 0x82E4, 1000 0010 1110 0100
    esc.write(1500);
    // 1000 0010 1110 0100
    // 1000
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(1));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(2));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(3));
    // 0010
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(4));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(5));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(6));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(7));
    // 1110
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(8));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(9));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(10));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(11));
    //0100
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(12));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(13));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(14));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16));

    const uint16_t frame2000 = ESC_DShot::dShotShiftAndAddChecksum(ESC_DShot::dShotConvert(2000));
    TEST_ASSERT_EQUAL(65518, frame2000); // 0xFFEE, 1111 1111 1110 1110
    esc.write(2000);
    // 1111
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(0));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(1));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(2));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(3));
    // 1110
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(12));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(13));
    TEST_ASSERT_EQUAL(HI, esc.getBufferItem(14));
    TEST_ASSERT_EQUAL(LO, esc.getBufferItem(15));
    TEST_ASSERT_EQUAL(0, esc.getBufferItem(16));
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_dshot);
    RUN_TEST(test_dshot_write);
    RUN_TEST(test_dshot_write_channel_b);

    UNITY_END();
}
