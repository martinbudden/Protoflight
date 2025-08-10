#include "DShotCodec.h"
#include "ESC_DShot.h"

#include <array>

#if defined(FRAMEWORK_RPI_PICO)

#include "pio/dshot_bidir_300.pio.h"
#include "pio/dshot_bidir_600.pio.h"

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

/*
See:
https://betaflight.com/docs/wiki/guides/current/DSHOT-RPM-Filtering
https://brushlesswhoop.com/dshot-and-bidirectional-dshot/

https://github.com/josephduchesne/pico-dshot-bidir
https://github.com/bastian2001/pico-bidir-dshot
https://github.com/symonb/Bidirectional-DSHOT-and-RPM-Filter?tab=readme-ov-file

For ESP32:
https://github.com/wjxway/ESP32-Bidirectional-DShot/blob/main/src/MotorCtrl.cpp (requires 2 wires)

*/

ESC_DShot::ESC_DShot(protocol_e protocol, uint16_t motorPoleCount) :
    _protocol(protocol),
    _motorPoleCount(motorPoleCount)
{
    setProtocol(protocol);
    constexpr float SECONDS_PER_MINUTE = 60.0F;
    _eRPMtoHz = 2.0F * (100.0F / SECONDS_PER_MINUTE) / static_cast<float>(_motorPoleCount);
}

void ESC_DShot::init(uint16_t pin)
{
    _pin = pin;

#if defined(FRAMEWORK_RPI_PICO)
#if defined(USE_DSHOT_RPI_PICO_PIO)
    if (_protocol == ESC_PROTOCOL_DSHOT300) {
        const bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_bidir_300_program, &_pio, &_pioStateMachine, &_pioOffset, pin, 1, true);
        hard_assert(success);
        // Configure it to run our program, and start it, using the
        // helper function we included in our .pio file.
        //printf("Using gpio %d\n", pin);
        dshot_bidir_300_program_init(_pio, _pioStateMachine, _pioOffset, pin, _cpuFrequency);
    } else {
        const bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_bidir_600_program, &_pio, &_pioStateMachine, &_pioOffset, pin, 1, true);
        hard_assert(success);
        // Configure it to run our program, and start it, using the
        // helper function we included in our .pio file.
        //printf("Using gpio %d\n", pin);
        dshot_bidir_600_program_init(_pio, _pioStateMachine, _pioOffset, pin, _cpuFrequency);
    }
    // The PIO State Machine is now running, to use we push onto its TX FIFO and pull from the RX FIFO
#else
    gpio_set_function(pin, GPIO_FUNC_PWM); // Set the pin to be PWM

    // RP2040 has 8 slices, RP2350 has 12 slices
    // Each slice can drive or measure 2 PWM signals, ie has 2 channels, PWM_CHAN_A and PWM_CHAN_B
    enum { PWM_CHANNEL_A = 0, PWM_CHANNEL_B =  1};

    // get PWM channel for the pin
    const uint32_t pwmChannel = pwm_gpio_to_channel(_pin);
    // channel B uses high order bits on RPI Pico
    _useHighOrderBits = pwmChannel == PWM_CHANNEL_B ? true : false;

    // Setup the PWM
    const uint32_t slice = pwm_gpio_to_slice_num(_pin);
    pwm_set_wrap(slice, _wrapCycleCount);
    pwm_set_enabled(slice, true); // start the PWM

    // Setup the DMA
    enum { PANIC_IF_NONE_AVAILABLE = true };
    _dmaChannel = dma_claim_unused_channel(PANIC_IF_NONE_AVAILABLE);

    dma_channel_config dmaConfig = dma_channel_get_default_config(_dmaChannel); // NOLINT(cppcoreguidelines-init-variables) false positive
    channel_config_set_dreq(&dmaConfig, pwm_get_dreq(slice)); // Set the DMA Data Request (DREQ)
    // transfer 32 bits at a time
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
#endif // USE_DSHOT_RPI_PICO_PIO
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
    TxH+TxL = 1250ns +/- 600ns (T0H + T0L or T1H + T1L)

    DShot150 means 150 kilobytes/second
    DShot 150 specification is
    T0H = 2500ns (data low pulse width)
    T0L = 4180ns (data low gap width)
    T1H = 5000ns (data high pulse width)
    T1L = 1680ns (data high gap width)
    TxH+TxL = 6680ns  (T0H + T0L or T1H + T1L)

    DShot 300 specification is
    T0H = 1250ns (data low pulse width)
    T0L = 2090ns (data low gap width)
    T1H = 2500ns (data high pulse width)
    T1L =  840ns (data high gap width)
    TxH+TxL = 3340ns  (T0H + T0L or T1H + T1L)

    see https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
    DShot 600 specification is
    T0H =  625ns (data low pulse width)
    T0L = 1045ns (data low gap width)
    T1H = 1250ns (data high pulse width)
    T0L =  420ns (data hig gap width)
    TxH+TxL = 1670ns  (T0H + T0L or T1H + T1L)
*/

    _protocol = protocol;
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

#if defined(FRAMEWORK_RPI_PICO) && !defined(USE_DSHOT_RPI_PICO_PIO)
    if (_pin != PIN_NOT_SET) {
        // the pin has already been set, so we need to re-set the wrap value
        const uint32_t slice = pwm_gpio_to_slice_num(_pin);
        pwm_set_enabled(slice, false); // stop the PWM
        pwm_set_wrap(slice, _wrapCycleCount);
        pwm_set_enabled(slice, true); // restart the PWM
    }
#endif
}

uint32_t ESC_DShot::nanoSecondsToCycles(uint32_t nanoSeconds) const
{
    // note: the k values cancel out, but give greater precision in the calculation
    const uint64_t k = 128;
    const uint64_t d = k * 1000000000L  / _cpuFrequency;
    const uint64_t ret = nanoSeconds * k / d;
    return static_cast<uint32_t>(ret);
}

/*!
value should be in the range [1000,2000]

Unidirectional DShot can use the hardware PWM generators and DMA.

For bidirectional DShot we need to wait for the response from the DMA and it seems bit-banging is required.
See [ESC BDShot protocol implementation](https://symonb.github.io/docs/drone/ESC/ESC_prot_impl_2_2#esc-bdshot-protocol-implementation)
for description and STM32 implementation.

On Raspberry Pi Pico we can use the Programmable IO (PIO) for this bit-banging.
*/
void ESC_DShot::write(uint16_t value) // NOLINT(readability-make-member-function-const)
{
#if defined(USE_DSHOT_RPI_PICO_PIO)
    // use the value to create a bidirectional DShot frame and send it to the PIO state machin
    value = DShotCodec::dShotConvert(value); // converts from range [1000,2000] to range [47,2047]
    value = DShotCodec::frameBidirectional(value);
    pio_sm_put(_pio, _pioStateMachine, value);
#else
    // set up a unidirectional DShot frame for sending via DMA
    value = DShotCodec::dShotConvert(value); // converts from range [1000,2000] to range [47,2047]
    const uint16_t frame = DShotCodec::frameUnidirectional(value);

    uint16_t maskBit = 1U << (DSHOT_BIT_COUNT - 1); // NOLINT(misc-const-correctness,hicpp-signed-bitwise)
    if (_useHighOrderBits) {
        for (auto& item : _dmaBuffer) {
            item = ((frame & maskBit) ? _dataHighPulseWidth : _dataLowPulseWidth) << 16;
            maskBit >>= 1U;
        }
    } else {
        for (auto& item : _dmaBuffer) {
            item = (frame & maskBit) ? _dataHighPulseWidth : _dataLowPulseWidth;
            maskBit >>= 1U;
        }
    }
    _dmaBuffer[DMA_BUFFER_SIZE - 1] = 0; // zero last value,  array size is DSHOT_BIT_COUNT + 1
#endif

#if defined(FRAMEWORK_RPI_PICO)
#if !defined(USE_DSHOT_RPI_PICO_PIO)
    // transfer DMA buffer to PWM
    dma_channel_set_trans_count(_dmaChannel, DMA_BUFFER_SIZE, DONT_START_YET);
    dma_channel_set_read_addr(_dmaChannel, &_dmaBuffer[0], START_IMMEDIATELY);
#endif
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#endif // FRAMEWORK
}

bool ESC_DShot::read()
{
    DShotCodec::telemetry_type_e telemetryType {};
    uint32_t value {};

#if defined(USE_DSHOT_RPI_PICO_PIO)
    const int32_t fifoCount = pio_sm_get_rx_fifo_level(_pio, _pioStateMachine);
    if (fifoCount >= 2) {
        // get DShot telemetry value from the PIO
        //value = pio_sm_get(_pio, _pioStateMachine);
        uint64_t samples = static_cast<uint64_t>(pio_sm_get_blocking(_pio, _pioStateMachine)) << 32;
        samples |= static_cast<uint64_t>(pio_sm_get_blocking(_pio, _pioStateMachine));
        value = DShotCodec::decodeSamples(samples, telemetryType);
    } else {
        return false;
    }
#else
    //!! TODO: get samples from bit banging
    std::array<uint32_t, 106> samples {};
    const uint32_t count = 10;
    value = DShotCodec::decodeSamples(&samples[0], count, telemetryType);
#endif

    ++_telemetryReadCount;
    switch (telemetryType) {
    case DShotCodec::TELEMETRY_INVALID:
        ++_telemetryErrorCount;
        return false;
    case DShotCodec::TELEMETRY_TYPE_ERPM: {
        // Convert to eRPM * 100
        //return ((1000000 * 60 / 100) + value / 2) / value;
        enum { ONE_MINUTE_IN_MICROSECONDS = 60000000 };
        // value is eRPM period in microseconds
        _eRPM = static_cast<int32_t>(ONE_MINUTE_IN_MICROSECONDS / value);
        break;
    }
    case DShotCodec::TELEMETRY_TYPE_TEMPERATURE:
        [[fallthrough]];
    case DShotCodec::TELEMETRY_TYPE_VOLTAGE:
        [[fallthrough]];
    case DShotCodec::TELEMETRY_TYPE_CURRENT:
        [[fallthrough]];
    case DShotCodec::TELEMETRY_TYPE_DEBUG1:
        [[fallthrough]];
    case DShotCodec::TELEMETRY_TYPE_DEBUG2:
        [[fallthrough]];
    case DShotCodec::TELEMETRY_TYPE_STRESS_LEVEL:
        [[fallthrough]];
    case DShotCodec::TELEMETRY_TYPE_STATE_EVENTS:
        [[fallthrough]];
    default:
        break;
    }
    return true;
}

void ESC_DShot::end()
{
#if defined(FRAMEWORK_RPI_PICO)
#if defined(USE_DSHOT_RPI_PICO_PIO)
    if (_protocol == ESC_PROTOCOL_DSHOT300) {
        pio_remove_program_and_unclaim_sm(&dshot_bidir_300_program, _pio, _pioStateMachine, _pioOffset);
    } else {
        pio_remove_program_and_unclaim_sm(&dshot_bidir_600_program, _pio, _pioStateMachine, _pioOffset);
    }
#endif
#elif defined(FRAMEWORK_ESPIDF)
    ESP_ERROR_CHECK(rmt_disable(_txChannel));
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#if !defined(UNIT_TEST)
    digitalWrite(_pin, LOW);
#endif
#endif // FRAMEWORK
}
