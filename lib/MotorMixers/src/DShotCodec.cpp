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
    0b00001, // 1 consecutive bits
    0b00011, // 2 consecutive bits
    0b00111, // 3 consecutive bits
    0b01111, // 4 consecutive bits
    0b11111  // 5 consecutive bits
};

// array to map 5-bit GCR quintet onto 4-bit nibble
const std::array<uint32_t, 32> DShotCodec::decodeQuintet = {
    0, 0,  0,  0, 0,  0,  0,  0,
    0, 9, 10, 11, 0, 13, 14, 15,
    0, 0,  2,  3, 0,  5,  6,  7,
    0, 0,  8,  1, 0,  4, 12,  0
};


// NOLINTBEGIN(hicpp-signed-bitwise)
uint16_t DShotCodec::dShotShiftAndAddChecksum(uint16_t value)
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
