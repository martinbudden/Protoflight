#include "dynamic_notch_filter.h"

#include <debug.h>
#include <time_microseconds.h>
#if (__cplusplus >= 202002L)
#include <ranges>
#endif


// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

DynamicNotchFilter::DynamicNotchFilter(float looptime_seconds) :
    _looptime_seconds(looptime_seconds)
{
}

/*!
Set and initialize.

The upper limit of the dynamic notch is the Nyquist frequency, that is sampleRate/2.

*/
void DynamicNotchFilter::set_config(const dynamic_notch_filter_config_t& config)
{
    _config = config;

    const float looprate_hz = 1.0F / _looptime_seconds;

    // Disable dynamic notch filter if update() would run at less than 2kHz
    static constexpr float DYN_NOTCH_UPDATE_MIN_HZ = 2000.F;
    if (looprate_hz < DYN_NOTCH_UPDATE_MIN_HZ) {
        _notch_count = 0;
        return;
    }

    _max_center_frequency_hz = 0.0F;
    static constexpr int STATE_MACHINE_ITERATION_COUNT = XYZ_AXIS_COUNT * static_cast<int>(STEP_COUNT);
    _filter_looptime_seconds = STATE_MACHINE_ITERATION_COUNT * _looptime_seconds;

    const float nyquist_frequency_hz = looprate_hz / 2.0F;
    _min_hz = config.dyn_notch_min_hz;
    _max_hz = std::fmaxf(_min_hz, config.dyn_notch_max_hz);
    _max_hz = std::fminf(_max_hz, nyquist_frequency_hz); // Ensure to not go above the nyquist limit

    _q = static_cast<float>(config.dyn_notch_q) / 100.0F;
    _notch_count = config.dyn_notch_count;

    _sample_count = std::max(static_cast<size_t>(1), static_cast<size_t>(nyquist_frequency_hz / _max_hz)); // max_hz = 600 & looprate_hz = 8000 -> sample_count = 6
    _sample_count_reciprocal = 1.0F / static_cast<float>(_sample_count);

    _sample_rate_hz = looprate_hz / static_cast<float>(_sample_count);

    _bin_resolution_hz = _sample_rate_hz / static_cast<float>(SDFT_SAMPLE_COUNT); // 18.5Hz per bin at 8kHz looptime and 600Hz max_hz
    _start_bin = static_cast<size_t>(lrintf(_min_hz / _bin_resolution_hz));
    if (_start_bin == 0) {
        _start_bin = 1; // cannot use bin 0 because it is for DC component
    }

    _end_bin = static_cast<size_t>(lrintf(_max_hz / _bin_resolution_hz));
    if (_end_bin > SDFT_BIN_COUNT - 1) {
        _end_bin = SDFT_BIN_COUNT - 1; // cannot use more than SDFT_BIN_COUNT bins
    }

    for (size_t axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
        _sdft[axis].init(_start_bin, _end_bin, _sample_count);
        for (size_t notch_index = 0; notch_index < _notch_count; ++notch_index) {
            // any init value is fine, but evenly spreading center frequencies across frequency range makes notches stick to peaks quicker
            _center_frequency_hz[axis][notch_index] =  _min_hz + (static_cast<float>(notch_index) + 0.5F) * (_max_hz - _min_hz) / static_cast<float>(_notch_count);
            _notch_filters[axis][notch_index].init_notch(_center_frequency_hz[axis][notch_index], _looptime_seconds, _q);
        }
    }
}

/*
Collect gyro data, to be analyzed in update() function

Date is averaged over sample_count values before sending to SDFT.

sample_count depends on looprate and max_hz.

For 8kHz looprate, Nyquist frequency is 4kHz,
so for max_hz = 600, sample_count is int(4000/600) = 6.
*/
void DynamicNotchFilter::push(const xyz_t& sample)
{
    _sample_sum += sample;
    if (_sample_index == _sample_count) {
        _sample_index = 0;
        // calculate mean value of accumulated samples
        _sample_average = _sample_sum * _sample_count_reciprocal;
        _sample_sum = { 0.0F, 0.0F, 0.0F };
    }

    // SDFT processing in batches to synchronize with incoming downsampled data
    _sdft[X].push(_sample_average.x, _sample_index);
    _sdft[Y].push(_sample_average.y, _sample_index);
    _sdft[Z].push(_sample_average.z, _sample_index);
    ++_sample_index;
}

/*!
Find frequency peaks and update filters
Calculation of filters frequencies takes 4 iterations per axis,
so each filter has its frequency set every 12 calls of this function

At 8kHz AHRS loop rate this gives 8kHz/12 = 666Hz, that is frequency update every 1.5ms.
At 4kHz AHRS loop rate this gives 4kHz/12 = 333Hz, that is frequency update every 3ms.
*/
void DynamicNotchFilter::update_notch_frequencies(Debug& debug) // NOLINT(readability-function-cognitive-complexity)
{
    const uint32_t startTimeUs = (debug.get_mode() == DEBUG_FFT_TIME) ? time_us() : 0;

    debug.set(DEBUG_FFT_TIME, 0, _state.step);

    // run an iteration of the state machine
    switch (_state.step) {
    case STEP_WINDOW:
        _sdft[_state.axis].calculate_window_squared(&_sdft_data[0]); // calculate power spectral density and put it in _sdft_data[]
        // Get total vibrational power in dyn notch range for noise floor estimate in STEP_CALC_FREQUENCIES
        _noise_threshold = 0.0F;
        for (size_t bin = _start_bin; bin <= _end_bin; ++bin) {
            _noise_threshold += _sdft_data[bin];
        }
        debug.set(DEBUG_FFT_TIME, 1, static_cast<int16_t>(time_us() - startTimeUs));
        break;
    case STEP_DETECT_PEAKS:
        // Get memory ready for new peak data on current axis
        _peaks.fill({0, 0.0F});
        // Search for N biggest peaks in frequency spectrum
        for (size_t bin = _start_bin + 1; bin < _end_bin; ++bin) {
            // Check if bin is peak
            if ((_sdft_data[bin] > _sdft_data[bin - 1]) && (_sdft_data[bin] > _sdft_data[bin + 1])) {
                // Check if peak is big enough to be one of_notch_count biggest peaks.
                // If so, insert into _peaks keeping _peaks sorted in descending VALUE order
                for (size_t notch_index = 0; notch_index < _notch_count; ++notch_index) {
                    if (_sdft_data[bin] > _peaks[notch_index].value) {
                        for (size_t k = _notch_count - 1; k > notch_index; --k) {
                            _peaks[k] = _peaks[k - 1];
                        }
                        _peaks[notch_index] = {.bin=bin, .value=_sdft_data[bin]};
                        break;
                    }
                }
                ++bin; // if bin is peak, next bin can't be peak so skip
            }
        }
        // Sort peaks into ascending BIN order (eg: 3, 8, 25, 0, 0, ..., 0) so we can later use interpolation between bins
        for (size_t notch_index = _notch_count - 1; notch_index > 0; --notch_index) {
            for (size_t k = 0; k < notch_index; ++k) {
                // Swap peaks but ignore swapping empty peaks (bin = 0). This leaves
                // empty peaks at the end of peaks array without moving them
                if (_peaks[k].bin > _peaks[k + 1].bin && _peaks[k + 1].bin != 0) {
                    const peak_t temp = _peaks[k];
                    _peaks[k] = _peaks[k + 1];
                    _peaks[k + 1] = temp;
                }
            }
        }
        debug.set(DEBUG_FFT_TIME, 1, static_cast<int16_t>(time_us() - startTimeUs));
        break;
    case STEP_CALCULATE_FREQUENCIES:
        // calculate _noise_threshold (= average power spectral density in dynamic notch range, excluding peaks)
        // _noise_threshold was initialized in STEP_WINDOW
        {
            size_t peak_count = 0;
            for (size_t notch_index = 0; notch_index < _notch_count; ++notch_index) {
                const size_t peakBin = _peaks[notch_index].bin;
                if (peakBin != 0) {
                    _noise_threshold -= _sdft_data[peakBin] + 0.75F*(_sdft_data[peakBin - 1] + _sdft_data[peakBin + 1]);
                    ++peak_count;
                }
            }
            _noise_threshold /= static_cast<float>(_end_bin - _start_bin - peak_count + 1);
            // A noise threshold twice the noise floor prevents peak tracking being too sensitive to noise
            _noise_threshold *= 2.0F;
        }
        for (size_t notch_index = 0; notch_index < _notch_count; ++notch_index) {
            // Only update _center_frequency_hz if there is a non-void(ie bin!=0) peak and peak value is above noise threshold
            const auto& peak = _peaks[notch_index];
            if (peak.bin != 0 && peak.value > _noise_threshold) {
                auto bin = static_cast<float>(peak.bin);
                // get heights of peak bin (y1) and shoulder bins (y0, y2)
                const float y0 = _sdft_data[peak.bin - 1];
                const float y1 = _sdft_data[peak.bin];
                const float y2 = _sdft_data[peak.bin + 1];
                // interpolate to estimate peak position (fit parabola y(x) over y0, y1 and y2, solve dy/dx=0 for x)
                const float denominator = 2.0F*(y0 - 2.0F*y1 + y2);
                if (denominator != 0.0F) {
                    bin += (y0 - y2) / denominator;
                }
                // Convert bin to frequency: frequency = bin * binResolution (bin 0 is 0Hz)
                // resolution is 18.5Hz per bin at 8kHz looptime and 600Hz max_hz
                const float center_frequency_hz = std::clamp(bin*_bin_resolution_hz, _min_hz, _max_hz);
                // PowerTransfer1 style smoothing moves notch center frequencies rapidly towards big peaks and slowly away, up to 10x faster
                const float cutoff_multiplier = std::clamp(peak.value / _noise_threshold, 1.0F, 10.0F);
                if (_config.dyn_notch_smoothing) {
                    static constexpr float DYN_NOTCH_SMOOTH_HZ = 4.0F;
                    // calculate center frequency as filtered value of new and old values
                    const float gain = PowerTransferFilter1::gain_from_frequency(DYN_NOTCH_SMOOTH_HZ * cutoff_multiplier, _filter_looptime_seconds);
                    //Equivalently
                    //const float omega = 2.0F * PI_F*DYN_NOTCH_SMOOTH_HZ * cutoff_multiplier * _filter_looptime_seconds;
                    //const float gain =  omega/(omega + 1.0F);

                    // Finally update _center_frequency_hz with smoothing , this is effectively `state += gain*(input - state)`
                    _center_frequency_hz[_state.axis][notch_index] += gain * (center_frequency_hz - _center_frequency_hz[_state.axis][notch_index]);
                } else {
                    _center_frequency_hz[_state.axis][notch_index] = center_frequency_hz;
                }
            }
        }
        // calculate max_center_frequency for instrumentation (eg display on OSD)
        if (_throttle_absolute > DYN_NOTCH_MIN_THROTTLE) {
            for (size_t notch_index = 0; notch_index < _notch_count; ++notch_index) {
                _max_center_frequency_hz = std::fmaxf(_max_center_frequency_hz, _center_frequency_hz[_state.axis][notch_index]);
            }
        }
        debug.set(DEBUG_FFT_TIME, 1, static_cast<int16_t>(time_us() - startTimeUs));
        break;
    case STEP_UPDATE_FILTERS:
#if (__cplusplus >= 202002L)
        for (auto notch_index : std::views::iota(size_t{0}, size_t{_notch_count})) {
#else
        for (size_t notch_index = 0; notch_index < _notch_count; ++notch_index) {
#endif
            // Only update notch filter coefficients if the corresponding peak got its center frequency updated in the previous step
            if (_peaks[notch_index].bin != 0 && _peaks[notch_index].value > _noise_threshold) {
                // set_notch_frequency is a reasonably expensive function involving calculation of sin and cos
                _notch_filters[_state.axis][notch_index].set_notch_frequency(_center_frequency_hz[_state.axis][notch_index]);
            }
        }
        // we have done all 4 steps for the current axis, so advance to the next axis
        if (_state.axis == AXIS_LAST) { _state.axis = AXIS_FIRST; } else { ++_state.axis; }

        debug.set(DEBUG_FFT_TIME, 1, static_cast<int16_t>(time_us() - startTimeUs));
        break;
    }

    // advance to the next step
    if (_state.step == STEP_LAST) { _state.step = STEP_FIRST; } else { ++_state.step; }
}

/*!
Use all the notch filters to filter the value
*/
void DynamicNotchFilter::filter(xyz_t& value)
{
#if (__cplusplus >= 202002L)
    for (auto notch_index : std::views::iota(size_t{0}, size_t{_notch_count})) {
#else
    for (size_t notch_index = 0; notch_index < _notch_count; ++notch_index) {
#endif
        value.x = _notch_filters[X][notch_index].filter(value.x);
        value.y = _notch_filters[Y][notch_index].filter(value.y);
        value.z = _notch_filters[Z][notch_index].filter(value.z);
    }
}
// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
