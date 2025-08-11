#pragma once

#include <array>
#include <cstdint>


/*!
DShot Encoder/Decoder

DShot Frame Structure
The DShot Frame defines which information is at which position in the data stream:

    11 bit throttle(S): 2048 possible values.
        0 is reserved for disarmed.
        1 to 47 are reserved for special commands.
        48 to 2047 (2000 steps) are for the actual throttle value
    1 bit telemetry request(T) - if this is set, telemetry data is sent back via a separate channel
    4 bit checksum(C) aka CRC (Cyclic Redundancy Check) to validate the frame

This results in a 16 bit (2 byte) frame with the following structure:

   SSSSSSSSSSSTCCCC


eRPM Telemetry Frame Structure

The eRPM telemetry frame sent by the ESC in bidirectional DSHOT mode is a 16 bit value, in the format:
The encoding of the eRPM data is not as straight forward as the one of the throttle frame:

    eeemmmmmmmmmcccc

where m is the 9-bit mantissa and e is the 3 bit exponent and cccc the checksum.
The resultant value is the mantissa shifted left by the exponent.

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
    static inline uint16_t pwmToDShotClipped(uint16_t value) { return value > 2000 ? pwmToDShot(2000) : value > 1000 ? pwmToDShot(value) : 0; }

    // non-inverted Checksum for unidirectional DShot
    static inline uint16_t checksumUnidirectional(uint16_t value) { return (value ^ (value >> 4) ^ (value >> 8)) & 0x0F; }
    static inline bool checksumUnidirectionalIsOK(uint16_t value) { return checksumUnidirectional(value>>4) == (value & 0x0F); }
    // inverted Checksum for bidirectional DShot
    static inline uint16_t checksumBidirectional(uint16_t value) { return(~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F; }
    static inline bool checksumBidirectionalIsOK(uint16_t value) { return checksumBidirectional(value>>4) == (value & 0x0F); }

    static inline uint16_t frameUnidirectional(uint16_t value) {
        value <<= 1;
        return (value << 4) | checksumUnidirectional(value);
    }
    static inline uint16_t frameBidirectional(uint16_t value) {
        value <<= 1;
        return (value << 4) | checksumBidirectional(value);
    }

    static uint32_t decodeERPM(uint16_t value);
    static uint32_t decodeTelemetryFrame(uint16_t value, telemetry_type_e& telemetryType);
    static uint32_t decodeSamples(uint64_t value, telemetry_type_e& telemetryType);
    static uint32_t decodeSamples(const uint32_t* samples, uint32_t count, telemetry_type_e& telemetryType);

    // see [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
    // for a good description of these conversions
    static uint32_t eRPM_to_GCR20(uint16_t value);
    static uint32_t GR20_to_GCR21(uint32_t value);
    static inline uint32_t GCR21_to_GCR20(uint32_t value) { return (value ^ (value >> 1U)); }
    static uint16_t GCR20_to_eRPM(uint32_t value);
public:
    static const std::array<uint32_t, 17> gcrBitLengths;
    static const std::array<uint32_t, 6> gcrSetBits;

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
