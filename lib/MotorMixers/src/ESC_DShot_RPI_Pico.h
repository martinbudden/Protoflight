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
public:
    ESC_DShot(protocol_e protocol, uint64_t cpuFrequency) :
        _cpuFrequency(cpuFrequency)
    {
        setProtocol(protocol);
    }
    explicit ESC_DShot(protocol_e protocol) : ESC_DShot(protocol, F_CPU) {}
    ESC_DShot() : ESC_DShot(ESC_PROTOCOL_DSHOT300, F_CPU) {}
    void init(uint16_t pin);
public:
    enum { DSHOT_BIT_COUNT = 16 };
    void setProtocol(protocol_e protocol);
    void write(uint16_t pulse);

    int32_t getMotorRPM() const { return _motorRPM; }
    void end();
    uint32_t nanoSecondsToCycles(uint32_t nanoSeconds) const;
// static functions
    static inline uint16_t pwmToDShot(uint16_t v) { return static_cast<uint16_t>(((v - 1000) * 2) + 47); }
    static inline uint16_t dShotConvert(uint16_t pulse) { return pulse > 2000 ? pwmToDShot(2000) : pulse > 1000 ? pwmToDShot(pulse) : 0; }
    static  uint16_t dShotShiftAndAddChecksum(uint16_t value);
// for testing
    uint32_t getDataHighPulseWidth() const { return _dataHighPulseWidth; }
    uint32_t getDataLowPulseWidth() const { return _dataLowPulseWidth; }
    uint32_t getBufferItem(size_t index) const { return _dmaBuffer[index]; }
protected:
    uint64_t _cpuFrequency;
    uint32_t _pwmChannel {};
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
    int32_t _motorRPM {}; //!< motor RMP, calculated from telemetry data
    enum { DMA_BUFFER_SIZE = DSHOT_BIT_COUNT + 1 }; // extra 1 for terminating zero value
    std::array<uint32_t, DMA_BUFFER_SIZE> _dmaBuffer {};
};
