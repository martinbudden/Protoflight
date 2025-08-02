#include "ESC_DShot.h"

#if defined(FRAMEWORK_RPI_PICO)

#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO

#if !defined(UNIT_TEST_BUILD)
#include <Arduino.h>
#endif

#endif // FRAMEWORK


void ESC_DShot::init(uint16_t pin)
{
    _pin = pin;

#if defined(FRAMEWORK_RPI_PICO)

    gpio_set_function(pin, GPIO_FUNC_PWM); // Set the pin to be PWM

    // RP2040 has 8 slices, RP2350 has 12 slices
    // Each slice can drive or measure 2 PWM signals, ie has 2 channels, PWM_CHAN_A and PWM_CHAN_B

    // get PWM channel for the pin
    _pwmChannel = pwm_gpio_to_channel(_pin);

    // Setup the PWM
    const uint32_t slice = pwm_gpio_to_slice_num(_pin);
    pwm_set_wrap(slice, _wrapCycleCount);
    pwm_set_enabled(slice, true); // start the PWM

    // Setup the DMA
    enum { PANIC_IF_NONE_AVAILABLE = true };
    _dmaChannel = dma_claim_unused_channel(PANIC_IF_NONE_AVAILABLE);

    dma_channel_config dmaConfig = dma_channel_get_default_config(_dmaChannel); // NOLINT(cppcoreguidelines-init-variables) false positive
    channel_config_set_dreq(&dmaConfig, pwm_get_dreq(slice)); // Set the DMA Data Request (DREQ)
    // transfer 32-bits at a time
    // don't increment write address so we always transfer to the same PWM register.
    // increment read address so we pick up a new value each time
    channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_32);
    channel_config_set_write_increment(&dmaConfig, false);
    channel_config_set_read_increment(&dmaConfig, true);

    dma_channel_configure(
        _dmaChannel,
        &dmaConfig,
        &pwm_hw->slice[slice].cc, // write to PWM counter compare
        nullptr, // inital read address, set when DMA is started
        0, // transfer count, set when DMA is started
        DONT_START_YET
    );

#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
#if !defined(UNIT_TEST_BUILD)
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
#endif

#endif // FRAMEWORK
}

/*!
Sets the DShot protocol.
Calculate pulse widths for that protocol.

Called in construction, before init().
*/
void ESC_DShot::setProtocol(protocol_e protocol)
{
/*
    The DShot protocol is based on WS2812B (NeoPixel) protocol.

    For comparison with W2812B
    W2812B https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
    T0 is the width of the pulse, T1 is the width of gap to the next pulse
    T0H = 400ns +/- 150ns
    T1H = 800ns +/- 150ns
    T0L = 850ns +/- 150ns
    T1L = 450ns +/- 150ns
    TH+TL = 1250ns +/- 600ns (T0H + T0L or T1H + T1L)

    DShot150 means 150 kilobytes/second
    DShot 150 specification is
    T0H = 2500ns (data low pulse width)
    T1H = 5000ns (data high pulse width)
    TH+TL = 7500ns  (T0H + T0L or T1H + T1L)

    DShot 600 specification is
    T0H =  625ns
    T1H = 1250ns
    TH+TL = 1875ns  (T0H + T0L or T1H + T1L)
*/

#if defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_TEST)
    // DShot 150 specification
    enum { DSHOT150_T0H = 2500, DSHOT150_T1H = 5000, DSHOT150_T = 7500 };

    // _dataLowPulseWidth and _dataHighPulseWidth are in processor cycles
    // for RPI_PICO: default CPU frequency is 150MHz, that is 0.15GHz

    _dataLowPulseWidth = nanoSecondsToCycles(DSHOT150_T0H);    // =  375 ( 375 = 2500 * 0.15GHz)
    _dataHighPulseWidth = nanoSecondsToCycles(DSHOT150_T1H);   // =  750 ( 750 = 5000 * 0.15GHz)
    _wrapCycleCount = nanoSecondsToCycles(DSHOT150_T); // = 1125 (1125 = 7500 * 0.15GHz)

    switch (protocol) {
    case ESC_PROTOCOL_DSHOT150:
        break;
    case ESC_PROTOCOL_DSHOT300:
        _dataLowPulseWidth /= 2;
        _dataHighPulseWidth /= 2;
        _wrapCycleCount /= 2;
        break;
    case ESC_PROTOCOL_DSHOT600:
        [[fallthrough]];
    case ESC_PROTOCOL_PROSHOT:
        _dataLowPulseWidth /= 4;
        _dataHighPulseWidth /= 4;
        _wrapCycleCount /= 4;
        break;
    default:
        break;
    }

#if defined(FRAMEWORK_RPI_PICO)
    if (_pin != PIN_NOT_SET) {
        // the pin has already been set, so we need to re-set the wrap value
        const uint32_t slice = pwm_gpio_to_slice_num(_pin);
        pwm_set_enabled(slice, false); // stop the PWM
        pwm_set_wrap(slice, _wrapCycleCount);
        pwm_set_enabled(slice, true); // restart the PWM
    }
#endif
#else
    (void)protocol;
#endif
}

uint32_t ESC_DShot::nanoSecondsToCycles(uint32_t nanoSeconds) const
{
    // note: the k values cancel out, but give greater precision in the calculation
    const uint64_t k = 128;
    const uint64_t d = k * 1000000000  / _cpuFrequency;
    const uint64_t ret = nanoSeconds * k / d;
    return static_cast<uint32_t>(ret);
}

uint16_t ESC_DShot::dShotShiftAndAddChecksum(uint16_t value)
{
    value <<= 1U;

    // compute checksum
    uint16_t checksum = 0;
    uint16_t checksumData = value;

    checksum ^= checksumData;
    checksumData >>= 4U;

    checksum ^= checksumData;
    checksumData >>= 4U;

    checksum ^= checksumData;
    checksumData >>= 4U;

    checksum ^= checksumData;

    checksum &= 0xf; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,hicpp-signed-bitwise)

    // lower 4 bits is checksum
    return (value << 4U) | checksum; // NOLINT(hicpp-signed-bitwise)
}

void ESC_DShot::write(uint16_t pulse) // NOLINT(readability-make-member-function-const)
{
    const uint16_t value = dShotConvert(pulse);
    const uint16_t frame = dShotShiftAndAddChecksum(value);

    uint16_t maskBit = 1U << (DSHOT_BIT_COUNT - 1); // NOLINT(misc-const-correctness,hicpp-signed-bitwise) false positive
    if (_pwmChannel == 0) {
        for (auto& item : _dmaBuffer) {
            item = (frame & maskBit) ? _dataHighPulseWidth : _dataLowPulseWidth;
            maskBit >>= 1U;
        }
    } else {
        for (auto& item : _dmaBuffer) {
            // channel B uses high order bits on RPI Pico
            item = ((frame & maskBit) ? _dataHighPulseWidth : _dataLowPulseWidth) << 16;
            maskBit >>= 1U;
        }
    }
    _dmaBuffer[DMA_BUFFER_SIZE - 1] = 0; // zero last value,  array size is DSHOT_BIT_COUNT + 1

#if defined(FRAMEWORK_RPI_PICO)

    // transfer DMA buffer to PWM
    dma_channel_set_trans_count(_dmaChannel, DMA_BUFFER_SIZE, DONT_START_YET);
    dma_channel_set_read_addr(_dmaChannel, &_dmaBuffer[0], START_IMMEDIATELY);

#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#endif // FRAMEWORK
}

void ESC_DShot::end()
{
#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
    ESP_ERROR_CHECK(rmt_disable(_txChannel));
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if !defined(UNIT_TEST)
    digitalWrite(_pin, LOW);
#endif
#endif // FRAMEWORK
}
