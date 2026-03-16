#pragma once

#include <filters.h>
#include <sdft.h>

#include <xyz_type.h>


class Debug;


struct dynamic_notch_filter_config_t {
    uint16_t dyn_notch_min_hz;
    uint16_t dyn_notch_max_hz;
    uint16_t dyn_notch_q;
    uint8_t  dyn_notch_count;
    uint8_t  dyn_notch_smoothing;
};

class DynamicNotchFilter {
public:
    virtual ~DynamicNotchFilter() = default;
    DynamicNotchFilter(float looptime_seconds);
    DynamicNotchFilter(uint32_t looptime) = delete;
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

    struct peak_t {
        size_t bin;
        float value;
    };
    struct state_t {
        uint32_t step;
        size_t axis;
    };
public:
    void set_config(const dynamic_notch_filter_config_t& config);
    const dynamic_notch_filter_config_t& get_config() const { return _config; }
    void push(const xyz_t& sample);
    void update_notch_frequencies(Debug& debug);
    void filter(xyz_t& value);
    bool is_active() const { return _notch_count > 0; }
    float get_max_center_frequency() const { return _max_center_frequency_hz; }
    void reset_max_center_frequency() { _max_center_frequency_hz = 0.0F; }
    void set_throttle(float throttle) { _throttle_absolute = std::fabs(throttle); }
// for testing
#if defined(FRAMEWORK_TEST)
    const xyz_t& get_sample_sum() const { return _sample_sum; };
    const xyz_t& get_sample_average() const { return _sample_average; };
    size_t get_sample_count() const { return _sample_count; }
    size_t get_start_bin() const { return _start_bin; }
    size_t get_end_bin() const { return _end_bin; }
    float get_resolution_hz() const { return _bin_resolution_hz; }
    void set_state(state_t state) { _state = state; }
    const state_t& get_state() const { return _state; }
    std::array<peak_t, DYNAMIC_NOTCH_COUNT_MAX>& get_peaks() { return _peaks; }
    std::array<float, SDFT_BIN_COUNT>& getSdftData() { return _sdft_data; }
    const float* getCenter_frequency_hzX() const { return &_center_frequency_hz[X][0]; }
#endif
private:
    float _looptime_seconds;
    dynamic_notch_filter_config_t _config {};
    float _throttle_absolute {};
    float _q {};
    float _min_hz {};
    float _max_hz {};
    size_t _notch_count {};
    float _max_center_frequency_hz {}; //!< just used for display on OSD
    float _center_frequency_hz[XYZ_AXIS_COUNT][DYNAMIC_NOTCH_COUNT_MAX] {};
    BiquadFilter _notch_filters[XYZ_AXIS_COUNT][DYNAMIC_NOTCH_COUNT_MAX] {};

    size_t _sample_index {};
    size_t _sample_count {};
    float _sample_count_reciprocal {};
    xyz_t _sample_sum {};
    xyz_t _sample_average {}; // downsampled data for frequency analysis, gives less aliasing and noise

    float _sample_rate_hz {};
    float _bin_resolution_hz {};
    size_t _start_bin {};
    size_t _end_bin {};
    float _noise_threshold {};
    float _filter_looptime_seconds {};

    // parameters for peak detection and frequency analysis
    state_t _state {};
    std::array<peak_t, DYNAMIC_NOTCH_COUNT_MAX> _peaks;
    std::array<float, SDFT_BIN_COUNT> _sdft_data;
    std::array<Sdft<SDFT_SAMPLE_COUNT>, XYZ_AXIS_COUNT> _sdft;
};
