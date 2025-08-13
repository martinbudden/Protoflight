#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#if defined(FRAMEWORK_RPI_PICO)
#include "hardware/pio.h"
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
        ESC_PROTOCOL_W2818B, // able to drive W2818B (NeoPixel) for debug setups
        ESC_PROTOCOL_COUNT
    };
    enum { DEFAULT_MOTOR_POLE_COUNT = 14 };
public:
    ESC_DShot(protocol_e protocol, uint16_t motorPoleCount);
    explicit ESC_DShot(protocol_e protocol) : ESC_DShot(protocol, DEFAULT_MOTOR_POLE_COUNT) {}
    ESC_DShot() : ESC_DShot(ESC_PROTOCOL_DSHOT300, DEFAULT_MOTOR_POLE_COUNT) {}
    void init(uint16_t pin);
public:
    enum { DSHOT_BIT_COUNT = 16 };
    void setProtocol(protocol_e protocol);
    void write(uint16_t value); // value should be in the DShot range [47,2047]
    bool read();

    int32_t getMotorRPM() const { return 2 * _eRPM / _motorPoleCount; } // eRPM = RPM * poles/2, /2 due to pole pairs, not poles
    float getMotorHz() const { return static_cast<float>(_eRPM) * _eRPMtoHz; }
    void end();
    uint32_t nanoSecondsToCycles(uint32_t nanoSeconds) const;
// for testing
    void setUseHighOrderBits(bool useHighOrderBits) { _useHighOrderBits = useHighOrderBits; }
    uint32_t getDataHighPulseWidth() const { return _dataHighPulseWidth; }
    uint32_t getDataLowPulseWidth() const { return _dataLowPulseWidth; }
    uint32_t getBufferItem(size_t index) const { return _dmaBuffer[index]; }
protected:
    uint32_t _cpuFrequency {150000000};
    protocol_e _protocol;
    uint32_t _useHighOrderBits = 0;
    uint32_t _wrapCycleCount {};
#if defined(FRAMEWORK_RPI_PICO)
#if defined(USE_DSHOT_RPI_PICO_PIO)
    PIO _pio {};
    uint _pioStateMachine {};
    uint _pioOffset {};
#else
    enum { START_IMMEDIATELY = true, DONT_START_YET = false };
    uint32_t _dmaChannel {};
#endif // USE_DSHOT_RPI_PICO_PIO
#endif
    uint32_t _dataHighPulseWidth {};
    uint32_t _dataLowPulseWidth {};
    enum { PIN_NOT_SET = 0xFFFF };
    uint16_t _pin {PIN_NOT_SET};
    uint16_t _motorPoleCount {DEFAULT_MOTOR_POLE_COUNT}; //!< number of poles the motor has, used to calculate RPM from telemetry data
    float _eRPMtoHz {};
    int32_t _eRPM {}; //!< eRPM, ie not taking into account motor pole count
    uint32_t _telemetryReadCount {};
    uint32_t _telemetryErrorCount {};

    enum { DMA_BUFFER_SIZE = DSHOT_BIT_COUNT + 1 }; // extra 1 for terminating zero value
    std::array<uint32_t, DMA_BUFFER_SIZE> _dmaBuffer {};
};
