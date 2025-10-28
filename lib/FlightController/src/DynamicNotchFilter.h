#pragma once

#include <Filters.h>
#include <SDFT.h>
#if !defined(FRAMEWORK_TEST)
#include <Targets.h>
#endif
#include <TimeMicroseconds.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <xyz_type.h>

class Debug;


class DynamicNotchFilter {
public:
    virtual ~DynamicNotchFilter() = default;
    DynamicNotchFilter(Debug& debug, float looptimeSeconds);
    DynamicNotchFilter(Debug& debug, uint32_t looptime) = delete;
private:
    // DynamicNotchFilter is not copyable or moveable
    DynamicNotchFilter(const DynamicNotchFilter&) = delete;
    DynamicNotchFilter& operator=(const DynamicNotchFilter&) = delete;
    DynamicNotchFilter(DynamicNotchFilter&&) = delete;
    DynamicNotchFilter& operator=(DynamicNotchFilter&&) = delete;
public:
    enum { DYNAMIC_NOTCH_COUNT_MAX = 7 }; // up to 7 dynamic notch filters on each axis

    // use a sample count of 72 for Fourier Transform, this gives 36 frequency bins
    enum { SDFT_SAMPLE_COUNT = 72, SDFT_BIN_COUNT = SDFT_SAMPLE_COUNT/2 };

    // axes and steps for state machine
    enum axis_e { X, Y, Z, XYZ_AXIS_COUNT, AXIS_FIRST = X, AXIS_LAST = Z };
    enum step_e {
        STEP_WINDOW,
        STEP_DETECT_PEAKS,
        STEP_CALCULATE_FREQUENCIES,
        STEP_UPDATE_FILTERS,
        STEP_COUNT,
        STEP_FIRST = STEP_WINDOW,
        STEP_LAST = STEP_UPDATE_FILTERS
    };
    static constexpr float DYN_NOTCH_MIN_THROTTLE = 0.20F;

    struct config_t {
        uint16_t dyn_notch_min_hz;
        uint16_t dyn_notch_max_hz;
        uint16_t dyn_notch_q;
        uint8_t  dyn_notch_count;
        uint8_t  dyn_notch_smoothing;
    };
    struct peak_t {
        size_t bin;
        float value;
    };
    struct state_t {
        int step;
        size_t axis;
    };
public:
    void setConfig(const config_t& config);
    const config_t& getConfig() const { return _config; }
    void push(const xyz_t& sample);
    void updateNotchFrequencies();
    void filter(xyz_t& value);
    bool isActive() const { return _notchCount > 0; }
    float getMaxCenterFrequency() const { return _maxCenterFrequencyHz; }
    void resetMaxCenterFrequency() { _maxCenterFrequencyHz = 0.0F; }
    void setThrottle(float throttle) { _throttleAbsolute = std::fabs(throttle); }
// for testing
#if defined(FRAMEWORK_TEST)
    const xyz_t& getSampleSum() const { return _sampleSum; };
    const xyz_t& getSampleAverage() const { return _sampleAverage; };
    size_t getSampleCount() const { return _sampleCount; }
    size_t getStartBin() const { return _startBin; }
    size_t getEndBin() const { return _endBin; }
    float getResolutionHz() const { return _binResolutionHz; }
    void setState(state_t state) { _state = state; }
    const state_t& getState() const { return _state; }
    std::array<peak_t, DYNAMIC_NOTCH_COUNT_MAX>& getPeaks() { return _peaks; }
    std::array<float, SDFT_BIN_COUNT>& getSdftData() { return _sdftData; }
    const float* getCenterFrequencyHzX() const { return &_centerFrequencyHz[X][0]; }
#endif
private:
    Debug& _debug;
    float _looptimeSeconds;
    config_t _config {};
    float _throttleAbsolute {};
    float _q {};
    float _minHz {};
    float _maxHz {};
    size_t _notchCount {};
    float _maxCenterFrequencyHz {}; //!< just used for display on OSD
    float _centerFrequencyHz[XYZ_AXIS_COUNT][DYNAMIC_NOTCH_COUNT_MAX] {};
    BiquadFilter _notchFilters[XYZ_AXIS_COUNT][DYNAMIC_NOTCH_COUNT_MAX] {};

    size_t _sampleIndex {};
    size_t _sampleCount {};
    float _sampleCountReciprocal {};
    xyz_t _sampleSum {};
    xyz_t _sampleAverage {}; // downsampled data for frequency analysis, gives less aliasing and noise

    float _sampleRateHz {};
    float _binResolutionHz {};
    size_t _startBin {};
    size_t _endBin {};
    float _noiseThreshold {};
    float _filterLooptimeSeconds {};

    // parameters for peak detection and frequency analysis
    state_t _state {};
    std::array<peak_t, DYNAMIC_NOTCH_COUNT_MAX> _peaks;
    std::array<float, SDFT_BIN_COUNT> _sdftData;
    std::array<SDFT<SDFT_SAMPLE_COUNT>, XYZ_AXIS_COUNT> _sdft;
};
