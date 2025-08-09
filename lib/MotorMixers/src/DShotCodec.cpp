#include "DShotCodec.h"

const std::array<uint32_t, 17> DShotCodec::gcr_raw_bitlengths = {
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

const std::array<uint32_t, 6> DShotCodec::gcr_raw_set_bits = {
    0b00000, // 0 consecutive bits, not a valid lookup
    0b00001, // 1 consecutive bit
    0b00011, // 2 consecutive bits
    0b00111, // 3 consecutive bits
    0b01111, // 4 consecutive bits
    0b11111  // 5 consecutive bits
};



// NOLINTBEGIN(hicpp-signed-bitwise)
uint16_t DShotCodec::dShotShiftAndAddChecksumOriginal(uint16_t value)
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

    checksum &= 0xf; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    // lower 4 bits is checksum
    return (value << 4U) | checksum;
}

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
    // Convert to eRPM * 100
    return ((1000000 * 60 / 100) + value / 2) / value;
}

uint32_t DShotCodec::decodeTelemetry(uint16_t value, telemetry_type_e& telemetryType)
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

int32_t DShotCodec::decodeTelemetry(uint64_t value, telemetry_type_e& telemetryType)
{
    // telemetry data must start with a 0, so if the first bit is high, we don't have any data
    if (value & 0x8000000000000000L) {
        telemetryType = TELEMETRY_INVALID;
        return 0;
    }

    int consecutiveBitCount = 1;  // we always start with the MSB
    uint32_t currentBit = 0;
    int bitCount = 0;
    uint32_t gcr_result = 0;
    // starting at 2nd bit since we know our data starts with a 0
    // 56 samples @ 0.917us sample rate = 51.33us sampled
    // loop the mask from 2nd MSB to  LSB
    for (uint64_t mask = 0x4000000000000000; mask != 0; mask >>= 1) {
        if (((value & mask) != 0) != currentBit) {  
            // if the masked bit doesn't match the current string of bits then end the current string and flip currentBit
            // bitshift gcr_result by N, and
            gcr_result = gcr_result << gcr_raw_bitlengths[consecutiveBitCount];
            // then set N bits in gcr_result, if currentBit is 1
            if (currentBit) {
                gcr_result |= gcr_raw_set_bits[gcr_raw_bitlengths[consecutiveBitCount]];
            }
            bitCount += gcr_raw_bitlengths[consecutiveBitCount];
            // invert currentBit, and reset consecutiveBitCount
            currentBit = !currentBit;
            consecutiveBitCount = 1;  // first bit found in the string is the one we just processed
        } else {
            // otherwise increment consecutiveBitCount
            ++consecutiveBitCount;
            if (consecutiveBitCount > 16) {
                // invalid run length at the current sample rate (outside of gcr_raw_bitlengths table)
                telemetryType = TELEMETRY_INVALID;
                return 0;
            }
        }
    }

    // outside the loop, we still need to account for the final bits if the string ends with 1s
    // bitshift gcr_result by N, and
    gcr_result <<= gcr_raw_bitlengths[consecutiveBitCount];
    // then set set N bits in gcr_result, if currentBit is 1
    if (currentBit) {
        gcr_result |= gcr_raw_set_bits[gcr_raw_bitlengths[consecutiveBitCount]];
    }
    // count bitCount (for debugging)
    bitCount += gcr_raw_bitlengths[consecutiveBitCount];

    // GCR data should be 21 bits
    if (bitCount < 21) {
        telemetryType = TELEMETRY_INVALID;
        return 0;
    }

    // chop the GCR data down to just the 21 MSB
    gcr_result = gcr_result >> (bitCount - 21);

    // convert edge transition GCR back to binary GCR
    gcr_result = (gcr_result ^ (gcr_result >> 1));


#if false
    // break the 20 bit gcr result into 4 quintets for converting back to regular 16 bit binary
    uint8_t quintets[4] = { 
        (uint8_t)(gcr_result >> 15) & (uint8_t)0b11111,
        (uint8_t)(gcr_result >> 10) & (uint8_t)0b11111,
        (uint8_t)(gcr_result >>  5) & (uint8_t)0b11111,
        (uint8_t)(gcr_result      ) & (uint8_t)0b11111
    };

    for(int i=0; i<4; ++i) {
        const uint8_t nibble = quintetToNibble[quintets[i]];
        //const uint8_t nibble = decodeGCRNibble(quintets[i]);
        result = (result << 4) | (nibble & 0b1111);
    }
#else
    uint32_t result = quintetToNibble[gcr_result & 0x1F];
    result |= quintetToNibble[(gcr_result >> 5) & 0x1F] << 4;
    result |= quintetToNibble[(gcr_result >> 10) & 0x1F] << 8;
    result |= quintetToNibble[(gcr_result >> 15) & 0x1F] << 12;
#endif

    // check the CRC (4 least significant bits in result)
    const uint8_t crc = result & 0b1111;
    result >>= 4;  // remove the crc from the result
    const uint8_t calculatedCRC = (~(result ^ (result >> 4) ^ (result >> 8))) & 0x0F;
    if (crc != calculatedCRC) {
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


uint32_t DShotCodec::decodeGCR(const uint32_t timings[], uint32_t count) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
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

    uint32_t decodedValue = quintetToNibble[value & 0x1F];
    decodedValue |= quintetToNibble[(value >> 5) & 0x1F] << 4;
    decodedValue |= quintetToNibble[(value >> 10) & 0x1F] << 8;
    decodedValue |= quintetToNibble[(value >> 15) & 0x1F] << 12;

    uint32_t checkSum = decodedValue;
    checkSum ^= (checkSum >> 8); // xor bytes
    checkSum ^= (checkSum >> 4); // xor nibbles

    if ((checkSum & 0xF) != 0xF) {
        return TELEMETRY_INVALID;
    }

    return decodedValue >> 4;
}

uint32_t DShotCodec::eRPM_to_GCR20(uint16_t value)
{
    uint32_t ret = nibbleToQuintet[value & 0x1F];
    ret |= nibbleToQuintet[(value >> 4) & 0x1F] << 5;
    ret |= nibbleToQuintet[(value >> 8) & 0x1F] << 10;
    ret |= nibbleToQuintet[(value >> 12) & 0x1F] << 15;
    return ret;
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

uint16_t DShotCodec::GCR20_to_eRPM(uint32_t value)
{
    uint32_t ret = quintetToNibble[value & 0x1F];
    ret |= quintetToNibble[(value >> 5) & 0x1F] << 4;
    ret |= quintetToNibble[(value >> 10) & 0x1F] << 8;
    ret |= quintetToNibble[(value >> 15) & 0x1F] << 12;

    return static_cast<uint16_t>(ret);
}

// NOLINTEND(hicpp-signed-bitwise)
