#pragma once

#include <array>
#include <cstdint>


/*!
DShot Encoder/Decoder

TODO: functions have been directly copied from ESC_DShot and need renaming.
*/

class DShotCodec {
public:
    enum telemetry_type_e {
        TELEMETRY_TYPE_ERPM           = 0,
        TELEMETRY_TYPE_TEMPERATURE    = 1, // Temperature Celsius
        TELEMETRY_TYPE_VOLTAGE        = 2, // Voltage with a step size of 0.25V ie [0, 0.25 ..., 63.75]
        TELEMETRY_TYPE_CURRENT        = 3, // Current with a step size of 1A ie [0, 1, ..., 255]
        TELEMETRY_TYPE_DEBUG1         = 4,
        TELEMETRY_TYPE_DEBUG2         = 5,
        TELEMETRY_TYPE_STRESS_LEVEL   = 6,
        TELEMETRY_TYPE_STATE_EVENTS   = 7,
        TELEMETRY_TYPE_COUNT,
        TELEMETRY_INVALID = 0xFFFF
    };
public:
    static inline uint16_t pwmToDShot(uint16_t value) { return static_cast<uint16_t>(((value - 1000) * 2) + 47); }
    static inline uint16_t dShotConvert(uint16_t value) { return value > 2000 ? pwmToDShot(2000) : value > 1000 ? pwmToDShot(value) : 0; }

    static uint16_t dShotShiftAndAddChecksumOriginal(uint16_t value);
//csum1 = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F; // Non-Inverted Checksum
//csum2 = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F; // Inverted Checksum
    static inline uint16_t dShotShiftAndAddChecksum(uint16_t value) {
        value <<= 1U;
        const uint16_t checksum = (value ^ (value >> 4U) ^ (value >> 8U)) & 0x0F;
        // Shift and merge 12 bit value with 4 bit checksum
        value =((value & 0xFFF) << 4U) | checksum;
        return value;
    }
    static inline uint16_t dShotShiftAndAddChecksumInverted(uint16_t value) {
        value = (value << 1U) | 1U;
        // slightly different checksum for inverted dshot
        const uint16_t checksum = (~(value ^ (value >> 4U) ^ (value >> 8U))) & 0x0F;
        // Shift and merge 12 bit value with 4 bit checksum
        value =((value & 0xFFF) << 4U) | checksum;
        return value;
    }
    static uint32_t decodeERPM(uint16_t value);
    static uint32_t decodeTelemetry(uint16_t value, telemetry_type_e& telemetryType);
    static int32_t decodeTelemetry(uint64_t value, telemetry_type_e& telemetryType);
    static uint32_t decodeGCR(const uint32_t timings[], uint32_t count);

    // see [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
    // for a good description of these conversions
    static uint32_t eRPM_to_GCR20(uint16_t value);
    static uint32_t GR20_to_GCR21(uint32_t value);
    static inline uint32_t GR21_to_GCR20(uint32_t value) { return (value ^ (value >> 1U)); }
    static uint16_t GCR20_to_eRPM(uint32_t value);
public:
    static const std::array<uint32_t, 17> gcr_raw_bitlengths;
    static const std::array<uint32_t, 6> gcr_raw_set_bits;

    // array to map 5-bit GCR quintet to 4-bit nibble
    static constexpr std::array<uint32_t, 32> quintetToNibble = {
        0, 0,  0,  0, 0,  0,  0,  0,
        0, 9, 10, 11, 0, 13, 14, 15,
        0, 0,  2,  3, 0,  5,  6,  7,
        0, 0,  8,  1, 0,  4, 12,  0
    };
    // array to map 4-bit nibble to 5-bit GCR quintet
    static constexpr std::array<uint8_t, 16> nibbleToQuintet = {
        0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
        0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F
    };
};
