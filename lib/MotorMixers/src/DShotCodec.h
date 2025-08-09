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
    static  uint16_t dShotShiftAndAddChecksum(uint16_t value);
    static uint16_t dShotShiftAndAddChecksumInverted(uint16_t value) {
        value = (value << 1U) | 1U;
        // slightly different crc for inverted dshot
        const uint16_t checksum = (~(value ^ (value >> 4U) ^ (value >> 8U))) & 0x0F;
        // Shift and merge 12 bit value with 4 bit checksum
        value =((value & 0xFFF) << 4U) | checksum;
        return value;
    }
    static uint32_t decodeERPM(uint16_t value);
    static uint32_t decodeTelemetry(uint16_t value, telemetry_type_e& telemetryType);
    static int32_t decodeTelemetry(uint64_t value, telemetry_type_e& telemetryType);
    static uint32_t decodeGCR(const uint32_t timings[], uint32_t count);
protected:
    static const std::array<uint32_t, 17> gcr_raw_bitlengths;
    static const std::array<uint32_t, 6> gcr_raw_set_bits;
    static const std::array<uint32_t, 32> decodeQuintet;
};