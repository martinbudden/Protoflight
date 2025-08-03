#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#endif // FRAMEWORK


#if !defined(F_CPU)
#define F_CPU 150000000L // CPU frequency
#endif

class ESC_DShot {
public:
    enum protocol_e {
        ESC_PROTOCOL_DSHOT150,
        ESC_PROTOCOL_DSHOT300,
        ESC_PROTOCOL_DSHOT600,
        ESC_PROTOCOL_PROSHOT,
        ESC_PROTOCOL_COUNT
    };
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
    ESC_DShot(protocol_e protocol, uint16_t motorPoleCount);
    explicit ESC_DShot(protocol_e protocol) : ESC_DShot(protocol, 14) {}
    ESC_DShot() : ESC_DShot(ESC_PROTOCOL_DSHOT300, 14) {}
    void init(uint16_t pin);
public:
    enum { DSHOT_BIT_COUNT = 16 };
    void setProtocol(protocol_e protocol);
    void write(uint16_t pulse);

    int32_t getMotorRPM() const { return 2 * _eRPM / _motorPoleCount; } // eRPM = RPM * poles/2
    float getMotorHz() const { return static_cast<float>(_eRPM) * _eRPMtoHz; }
    void end();
    uint32_t nanoSecondsToCycles(uint32_t nanoSeconds) const;
// static functions
    static inline uint16_t pwmToDShot(uint16_t v) { return static_cast<uint16_t>(((v - 1000) * 2) + 47); }
    static inline uint16_t dShotConvert(uint16_t pulse) { return pulse > 2000 ? pwmToDShot(2000) : pulse > 1000 ? pwmToDShot(pulse) : 0; }
    static  uint16_t dShotShiftAndAddChecksum(uint16_t value);
    static uint32_t decodeERPM(uint16_t value);
    static uint32_t decodeTelemetry(uint16_t value, telemetry_type_e& telemetryType);
    static uint32_t decodeGCR(const uint32_t timings[], uint32_t count);
// for testing
    uint32_t getDataHighPulseWidth() const { return _dataHighPulseWidth; }
    uint32_t getDataLowPulseWidth() const { return _dataLowPulseWidth; }
    uint32_t getBufferItem(size_t index) const { return _dmaBuffer[index]; }
    void setUseHighOrderBits(bool useHighOrderBits) { _useHighOrderBits = useHighOrderBits; }
protected:
    uint64_t _cpuFrequency {150000000L};
    uint32_t _useHighOrderBits = 0;
    uint32_t _wrapCycleCount {};
#if defined(FRAMEWORK_RPI_PICO)
    enum { START_IMMEDIATELY = true, DONT_START_YET = false };
    uint32_t _dmaChannel {};
#endif
    uint32_t _dataHighPulseWidth {};
    uint32_t _dataLowPulseWidth {};
    enum { PIN_NOT_SET = 0xFFFF };
    uint16_t _pin {PIN_NOT_SET};
    uint16_t _motorPoleCount {14}; //!< number of poles the motor has, used to calculate RPM from telemetry data
    int32_t _eRPM {}; //!< eRPM, ie not taking into account motor pole count
    float _eRPMtoHz {};
    enum { DMA_BUFFER_SIZE = DSHOT_BIT_COUNT + 1 }; // extra 1 for terminating zero value
    std::array<uint32_t, DMA_BUFFER_SIZE> _dmaBuffer {};
};
