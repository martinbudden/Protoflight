#include "DShotCodec.h"

const std::array<uint32_t, 17> DShotCodec::gcrBitLengths = {
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

const std::array<uint32_t, 6> DShotCodec::gcrSetBits = {
    0b00000, // 0 consecutive bits, not a valid lookup
    0b00001, // 1 consecutive bit
    0b00011, // 2 consecutive bits
    0b00111, // 3 consecutive bits
    0b01111, // 4 consecutive bits
    0b11111  // 5 consecutive bits
};



// NOLINTBEGIN(hicpp-signed-bitwise)
uint32_t DShotCodec::decodeERPM(uint16_t value)
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
    return value;
}

uint32_t DShotCodec::decodeTelemetryFrame(uint16_t value, telemetry_type_e& telemetryType)
{
    // value is of form "eeem mmmm mmmm": e - exponent, m - mantissa
    // https://github.com/bird-sanctuary/extended-dshot-telemetry

    // eRPM frames are of form "0000 mmmm mmmm" or "eee1 mmmm mmmm"
    const uint16_t type = (value & 0x0F00) >> 8;
    const bool isErpm = (type & 0x01) || (type == 0);
    if (isErpm) {
        telemetryType = TELEMETRY_TYPE_ERPM;
        const uint16_t m = value & 0x01FF;
        const uint16_t e = (value & 0xFE00) >> 9;
        value = m << e;
        if (value == 0) {
            return TELEMETRY_INVALID;
        }
        return value;
    }
    // Extended DShot Telemetry (EDT) frame is of the form:
    // ppp0mmmmmmmm
    // where ppp is the telemetry type and mmmmmmmm is the value
    telemetryType = static_cast<telemetry_type_e>(type >> 1);
    return value & 0x00FF;
}

/*!
Decode samples returned by Raspberry Pi PIO implementation.

Returns the value of the Extended DShot Telemetry (EDT) frame (without the  checksum).
*/
uint32_t DShotCodec::decodeSamples(uint64_t value, telemetry_type_e& telemetryType)
{
    // telemetry data must start with a 0, so if the first bit is high, we don't have any data
    if (value & 0x8000000000000000L) {
        telemetryType = TELEMETRY_INVALID;
        return 0;
    }

    uint32_t consecutiveBitCount = 1;  // we always start with the MSB
    uint32_t currentBit = 0;
    uint32_t bitCount = 0;
    uint32_t gcr_result = 0;
    // starting at 2nd bit since we know our data starts with a 0
    // 56 samples @ 0.917us sample rate = 51.33us sampled
    // loop the mask from 2nd MSB to  LSB
    for (uint64_t mask = 0x4000000000000000; mask != 0; mask >>= 1) {
        if (((value & mask) != 0) != currentBit) {
            // if the masked bit doesn't match the current string of bits then end the current string and flip currentBit
            // bitshift gcr_result by N, and
            gcr_result = gcr_result << gcrBitLengths[consecutiveBitCount]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
            // then set N bits in gcr_result, if currentBit is 1
            if (currentBit) {
                gcr_result |= gcrSetBits[gcrBitLengths[consecutiveBitCount]]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
            }
            bitCount += gcrBitLengths[consecutiveBitCount]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
            // invert currentBit, and reset consecutiveBitCount
            currentBit = !currentBit;
            consecutiveBitCount = 1;  // first bit found in the string is the one we just processed
        } else {
            // otherwise increment consecutiveBitCount
            ++consecutiveBitCount;
            if (consecutiveBitCount > 16) {
                // invalid run length at the current sample rate (outside of gcrBitLengths table)
                telemetryType = TELEMETRY_INVALID;
                return 0;
            }
        }
    }

    // outside the loop, we still need to account for the final bits if the string ends with 1s
    // bitshift gcr_result by N, and
    gcr_result <<= gcrBitLengths[consecutiveBitCount]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    // then set set N bits in gcr_result, if currentBit is 1
    if (currentBit) {
        gcr_result |= gcrSetBits[gcrBitLengths[consecutiveBitCount]]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }
    // count bitCount (for debugging)
    bitCount += gcrBitLengths[consecutiveBitCount]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    // GCR data should be 21 bits
    if (bitCount < 21) {
        telemetryType = TELEMETRY_INVALID;
        return 0;
    }

    // chop the GCR data down to just the 21 most significant bits
    gcr_result = gcr_result >> (bitCount - 21);

    // convert 21-bit edge transition GCR to 20-bit binary GCR
    const uint32_t gcr20 = GCR21_to_GCR20(gcr_result);

    const uint16_t result = GCR20_to_eRPM(gcr20);

    if (!checksumBidirectionalIsOK(result)) {
        telemetryType = TELEMETRY_INVALID;
        return 0;
    }
    return decodeTelemetryFrame(result >> 4, telemetryType);
}


/*!
Decode samples value returned by bit-banging.

Returns the value of the Extended DShot Telemetry (EDT) frame (without the  checksum).
*/
uint32_t DShotCodec::decodeSamples(const uint32_t* samples, uint32_t count, telemetry_type_e& telemetryType) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
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
        const uint32_t diff = samples[ii] - samples[ii-1]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
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

    const uint16_t result = GCR20_to_eRPM(value);

    if (!checksumBidirectionalIsOK(result)) {
        telemetryType = TELEMETRY_INVALID;
        return 0;
    }
    return decodeTelemetryFrame(result >> 4, telemetryType);
}

uint16_t DShotCodec::GCR20_to_eRPM(uint32_t value)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    uint32_t ret = quintetToNibble[value & 0x1F];
    ret |= quintetToNibble[(value >> 5) & 0x1F] << 4;
    ret |= quintetToNibble[(value >> 10) & 0x1F] << 8;
    ret |= quintetToNibble[(value >> 15) & 0x1F] << 12;
    return static_cast<uint16_t>(ret);
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}

uint32_t DShotCodec::eRPM_to_GCR20(uint16_t value)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    uint32_t ret = nibbleToQuintet[value & 0x1F];
    ret |= nibbleToQuintet[(value >> 4) & 0x1F] << 5;
    ret |= nibbleToQuintet[(value >> 8) & 0x1F] << 10;
    ret |= nibbleToQuintet[(value >> 12) & 0x1F] << 15;
    return ret;
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}

uint32_t DShotCodec::GR20_to_GCR21(uint32_t value)
{
// Map the GCR to a 21 bit value, this new value starts with a 0 and the rest of the bits are set by the following two rules:
//    1. If the current input bit in GCR data is a 1 then the output bit is the inverse of the previous output bit
//    2. If the current input bit in GCR data is a 0 then the output bit is the same as the previous output

    uint32_t ret = 0;
    uint32_t previousOutputBit = 0;

    for (uint32_t mask = 1U << 20; mask != 0; mask >>= 1U) {
        ret <<= 1;
        const uint32_t inputBit = value & mask;
        const uint32_t outputBit = inputBit ? !previousOutputBit : previousOutputBit;
        previousOutputBit = outputBit;
        ret |= outputBit;
    }

    return ret;
}

// NOLINTEND(hicpp-signed-bitwise)
