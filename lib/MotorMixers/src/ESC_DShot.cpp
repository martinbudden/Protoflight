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
        dshot_bidir_300_program_init(_pio, _pioStateMachine, _pioOffset, pin);
    } else {
        const bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_bidir_600_program, &_pio, &_pioStateMachine, &_pioOffset, pin, 1, true);
        hard_assert(success);
        // Configure it to run our program, and start it, using the
        // helper function we included in our .pio file.
        //printf("Using gpio %d\n", pin);
        dshot_bidir_600_program_init(_pio, _pioStateMachine, _pioOffset, pin);
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

#if defined(FRAMEWORK_RPI_PICO)
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
#if defined(USE_DSHOT_RPI_PICO_PIO)
    pulse = (pulse << 1) | 1;
    // slightly different crc for inverted dshot
    const uint16_t checksum = (~(pulse ^ (pulse >> 4) ^ (pulse >> 8))) & 0x0F;
    // Shift and merge 12 bit pulse with 4 bit checksum
    pulse =((pulse & 0xFFF) << 4) | checksum;
    // put value to the PIO
    pio_sm_put(_pio, _pioStateMachine, pulse);
#else
    const uint16_t value = dShotConvert(pulse);
    const uint16_t frame = dShotShiftAndAddChecksum(value);

    uint16_t maskBit = 1U << (DSHOT_BIT_COUNT - 1); // NOLINT(misc-const-correctness,hicpp-signed-bitwise) false positive
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
    uint64_t value {};
#if defined(USE_DSHOT_RPI_PICO_PIO)
    const int32_t fifoCount = pio_sm_get_rx_fifo_level(_pio, _pioStateMachine);
    if (fifoCount >= 2) {
        // get DShot telemetry value from the PIO
        //value = pio_sm_get(_pio, _pioStateMachine);
        value = static_cast<uint64_t>(pio_sm_get_blocking(_pio, _pioStateMachine)) << 32;
        value |= static_cast<uint64_t>(pio_sm_get_blocking(_pio, _pioStateMachine));
    } else {
        return false;
    }
#endif
    ++_telemetryReadCount;
    telemetry_type_e telemetryType {};
    const uint32_t decoded = decodeTelemetry(value, telemetryType);
    switch (telemetryType) {
    case TELEMETRY_INVALID:
        ++_telemetryErrorCount;
        return false;
    case TELEMETRY_TYPE_ERPM:
        _eRPM = static_cast<int32_t>(decoded);
        break;
    case TELEMETRY_TYPE_TEMPERATURE:
        [[fallthrough]];
    case TELEMETRY_TYPE_VOLTAGE:
        [[fallthrough]];
    case TELEMETRY_TYPE_CURRENT:
        [[fallthrough]];
    case TELEMETRY_TYPE_DEBUG1:
        [[fallthrough]];
    case TELEMETRY_TYPE_DEBUG2:
        [[fallthrough]];
    case TELEMETRY_TYPE_STRESS_LEVEL:
        [[fallthrough]];
    case TELEMETRY_TYPE_STATE_EVENTS:
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

// NOLINTBEGIN(hicpp-signed-bitwise)
uint32_t ESC_DShot::decodeERPM(uint16_t value)
{
    // eRPM range
    if (value == 0x0FFF) {
        return 0;
    }
    const uint16_t m = value & 0x01FF;
    const uint16_t e = (value & 0xFE00) >> 9;
    value = m << e;
    if (value == 0) {
        return TELEMETRY_INVALID;
    }
    // Convert to eRPM * 100
    return ((1000000 * 60 / 100) + value / 2) / value;
}

uint32_t ESC_DShot::decodeTelemetry(uint16_t value, telemetry_type_e& telemetryType)
{
    // value is of form "eeem mmmm mmmm": e - exponent, m - mantissa
    // https://github.com/bird-sanctuary/extended-dshot-telemetry

    // eRPM frames are of form "0000 mmmm mmmm" or "eee1 mmmm mmmm"
    const uint16_t type = (value & 0x0F00) >> 8;
    const bool isErpm = (type & 0x01) || (type == 0);
    if (isErpm) {
        telemetryType = TELEMETRY_TYPE_ERPM;
        return decodeERPM(value);
    }
    // note: type >> 1 is in range [0..7]
    telemetryType = static_cast<telemetry_type_e>(type >> 1);
    return value & 0x00FF;
}

static const std::array<uint32_t, 17> gcr_raw_bitlengths = {
    0, // 0 consecutive bits, not a valid lookup
    1, // 1 consecutive bits
    1, // 2 consecutive bits
    1, // 3 consecutive bits
    2, // 4 consecutive bits
    2, // 5 consecutive bits
    2, // 6 consecutive bits
    3, // 7 consecutive bits
    3, // 8 consecutive bits
    3, // 9 consecutive bits
    3, //10 consecutive bits
    4, //11 consecutive bits, not valid, but sometimes occurs at the end of the string
    4, //12 consecutive bits
    4, //13 consecutive bits
    5, //14 consecutive bits
    5, //15 consecutive bits
    5, //16 consecutive bits
    // more than 10 consecutive samples, means 4 0 or 1 in a row is invalid in GCR
};

static const std::array<uint32_t, 6> gcr_raw_set_bits = {
    0b00000, // 0 consecutive bits, not a valid lookup
    0b00001, // 1 consecutive bits
    0b00011, // 2 consecutive bits
    0b00111, // 3 consecutive bits
    0b01111, // 4 consecutive bits
    0b11111  // 5 consecutive bits
};

// array to map 5-bit GCR quintet onto 4-bit nibble
static const std::array<uint32_t, 32> decodeQuintet = {
    0, 0,  0,  0, 0,  0,  0,  0,
    0, 9, 10, 11, 0, 13, 14, 15,
    0, 0,  2,  3, 0,  5,  6,  7,
    0, 0,  8,  1, 0,  4, 12,  0
};


int32_t ESC_DShot::decodeTelemetry(uint64_t value, telemetry_type_e& telemetryType)
{
    // telemetry data must start with a 0, so if the first bit is high, we don't have any data
    if (value & 0x8000000000000000L) {
        telemetryType = TELEMETRY_INVALID;
        return 0;
    }

    int consecutiveCount = 1;  // we always start with the MSB
    bool current = false;
    int bitCount = 0;
    uint32_t gcr_result = 0;
    // starting at 2nd bit since we know our data starts with a 0
    // 56 samples @ 0.917us sample rate = 51.33us sampled
    // loop the mask from 2nd MSB to  LSB
    for (uint64_t mask = 0x4000000000000000; mask != 0; mask >>= 1) {
        if (((value & mask) != 0) != current) {  // if the doesn't match the current string of bits, end the current string and flip current
            // bitshift gcr_result by N, and
            gcr_result = gcr_result << gcr_raw_bitlengths[consecutiveCount];
            // then set set N bits in gcr_result, if current is 1
            if (current) {
                gcr_result |= gcr_raw_set_bits[gcr_raw_bitlengths[consecutiveCount]];
            }
            // count bitCount (for debugging)
            bitCount += gcr_raw_bitlengths[consecutiveCount];
            // invert current, and reset consecutiveCount
            current = !current;
            consecutiveCount = 1;  // first bit found in the string is the one we just processed
        } else {  // otherwise increment consecutiveCount
            ++consecutiveCount;
            if (consecutiveCount > 16) {
                // invalid run length at the current sample rate (outside of gcr_raw_bitlengths table)
                telemetryType = TELEMETRY_INVALID;
                return 0;
            }
        }
    }

    // outside the loop, we still need to account for the final bits if the string ends with 1s
    // bitshift gcr_result by N, and
    gcr_result <<= gcr_raw_bitlengths[consecutiveCount];
    // then set set N bits in gcr_result, if current is 1
    if (current) {
        gcr_result |= gcr_raw_set_bits[gcr_raw_bitlengths[consecutiveCount]];
    }
    // count bitCount (for debugging)
    bitCount += gcr_raw_bitlengths[consecutiveCount];

    // GCR data should be 21 bits
    if (bitCount < 21) {
        telemetryType = TELEMETRY_INVALID;
        return 0;
    }

    // chop the GCR data down to just the 21 MSB
    gcr_result = gcr_result >> (bitCount - 21);

    // convert edge transition GCR back to binary GCR
    gcr_result = (gcr_result ^ (gcr_result >> 1));

    // Serial.print("GCR: "); print_bin(gcr_result);

#if false
    // break the 20 bit gcr result into 4 quintets for converting back to regular 16 bit binary
    uint8_t quintets[4] = { 
        (uint8_t)(gcr_result >> 15) & (uint8_t)0b11111,
        (uint8_t)(gcr_result >> 10) & (uint8_t)0b11111,
        (uint8_t)(gcr_result >>  5) & (uint8_t)0b11111,
        (uint8_t)(gcr_result      ) & (uint8_t)0b11111
    };

    for(int i=0; i<4; ++i) {
        const uint8_t nibble = decodeQuintet[quintets[i]];
        //const uint8_t nibble = decodeGCRNibble(quintets[i]);
        result = (result << 4) | (nibble & 0b1111);
    }
#else
    uint32_t result = decodeQuintet[gcr_result & 0x1F];
    result |= decodeQuintet[(gcr_result >> 5) & 0x1F] << 4;
    result |= decodeQuintet[(gcr_result >> 10) & 0x1F] << 8;
    result |= decodeQuintet[(gcr_result >> 15) & 0x1F] << 12;
#endif

    // check CRC! (4 LSB in result)
    const uint8_t result_crc = result & 0b1111;  //extract the telemetry nibble
    result >>= 4;  // and then remove it from the result
    const uint8_t calc_crc = (~(result ^ (result >> 4) ^ (result >> 8))) & 0x0F;
    if (result_crc != calc_crc) {
        telemetryType = TELEMETRY_INVALID;
        return 0;
    }

    // todo: properly handle zero eRPM value (min representable eRPM period)
    if (result & 0x100) {
        telemetryType = TELEMETRY_TYPE_ERPM;
        uint32_t eRPM_period_us = static_cast<uint32_t>(result) & 0b111111111;  // period base is 9 bits above 4 bit CRC
        eRPM_period_us <<= (result >> 9);  // period base is the left shift amount stored in the 3 MSB
        enum { ONE_MINUTE_IN_MICROSECONDS = 60000000 };
        return static_cast<int32_t>(ONE_MINUTE_IN_MICROSECONDS / eRPM_period_us); // eRPM
    }
    // extended telemetry
    telemetryType = static_cast<telemetry_type_e>(result >> 8);
    return static_cast<int32_t>(result & 0xFF); // bottom 8 bits
}


uint32_t ESC_DShot::decodeGCR(const uint32_t timings[], uint32_t count) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
{
    // decode 16 bit GCR (Group Coded Recording) Run Length Limited (RLL) encoding
    // https://en.wikipedia.org/wiki/Run-length_limited#GCR:_(0,2)_RLL

    // see also https://github.com/bird-sanctuary/arduino-bi-directional-dshot/blob/master/src/arduino_dshot.cpp

    // a 16-bit value is encoded as 21 bits in GCR:
    //   each 4-bit nibble is encoded as 5 bits, giving 20 bits
    //   this 20-bit value is then encoded into 21 bits
    //   see https://brushlesswhoop.com/dshot-and-bidirectional-dshot/ for an example

    uint32_t value = 0;
    uint32_t bitCount = 0;
    for (uint32_t ii = 1; ii < count; ++ii) {
        const uint32_t diff = timings[ii] - timings[ii-1]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        const uint32_t runLength = (diff + 8) / 16;
        value <<= runLength;
        value |= 1 << (runLength - 1);
        bitCount += runLength;
        if (bitCount >= 21) {
            break;
        }
    }
    if (bitCount < 21) {
        const uint32_t runLength = 21 - bitCount;
        value <<= runLength;
        value |= 1 << (runLength - 1);
        bitCount += runLength;
    }
    if (bitCount != 21) {
        return TELEMETRY_INVALID;
    }

    uint32_t decodedValue = decodeQuintet[value & 0x1F];
    decodedValue |= decodeQuintet[(value >> 5) & 0x1F] << 4;
    decodedValue |= decodeQuintet[(value >> 10) & 0x1F] << 8;
    decodedValue |= decodeQuintet[(value >> 15) & 0x1F] << 12;

    uint32_t checkSum = decodedValue;
    checkSum ^= (checkSum >> 8); // xor bytes
    checkSum ^= (checkSum >> 4); // xor nibbles

    if ((checkSum & 0xF) != 0xF) {
        return TELEMETRY_INVALID;
    }

    return decodedValue >> 4;
}
// NOLINTEND(hicpp-signed-bitwise)
