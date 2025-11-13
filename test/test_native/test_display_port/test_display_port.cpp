#include <Display.h>
#include <DisplayPortNull.h>
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
    static Display display(displayPortNull);
    (void)display;
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,readability-magic-numbers)

// See for example for testing GCR encoding https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625
// see also https://elmagnifico.tech/2023/04/07/bi-directional-DSHOT/
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(test_display);

    UNITY_END();
}
