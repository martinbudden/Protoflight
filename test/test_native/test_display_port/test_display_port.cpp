#include <DisplayPortNull.h>
#include <FormatInteger.h>
#include <array>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)
void test_display()
{
    //BUS_SPI::spi_pins_t spiPins{.cs=46,.sck=44,.cipo=43,.copi=14,.irq=0xFF};
    //static DisplayPortMax7456 displayPortMax7456(BUS_SPI::BUS_INDEX_0, spiPins);
    static DisplayPortNull displayPortNull;
    (void)displayPortNull;

}
void test_format()
{
    std::array<char, 32> buf {};

    formatFixed6point3(12345, &buf[0]);
    TEST_ASSERT_EQUAL_STRING("12.345", &buf[0]);

    formatFixed6point3(3698, &buf[0]);
    TEST_ASSERT_EQUAL_STRING(" 3.698", &buf[0]);

    formatFixed6point3(168, &buf[0]);
    TEST_ASSERT_EQUAL_STRING(" 0.168", &buf[0]);

    formatFixed6point3(27, &buf[0]);
    TEST_ASSERT_EQUAL_STRING(" 0.027", &buf[0]);

    formatFixed6point3(3, &buf[0]);
    TEST_ASSERT_EQUAL_STRING(" 0.003", &buf[0]);
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_display);
    RUN_TEST(test_format);

    UNITY_END();
}
