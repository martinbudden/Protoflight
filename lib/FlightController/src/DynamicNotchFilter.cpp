#include "DynamicNotchFilter.h"

#include <Debug.h>
#include <cmath>


// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

DynamicNotchFilter::DynamicNotchFilter(Debug& debug, float looptimeSeconds) :
    _debug(debug),
    _looptimeSeconds(looptimeSeconds)
{
}

/*!
Set and initialize.

The upper limit of the dynamic notch is the Nyquist frequency, that is sampleRate/2.

*/
void DynamicNotchFilter::setConfig(const config_t& config)
{
    _config = config;

    const float looprateHz = 1.0F / _looptimeSeconds;

    // Disable dynamic notch filter if update() would run at less than 2kHz
    static constexpr float DYN_NOTCH_UPDATE_MIN_HZ = 2000.F;
    if (looprateHz < DYN_NOTCH_UPDATE_MIN_HZ) {
        _notchCount = 0;
        return;
    }

    _maxCenterFrequencyHz = 0.0F;
    static constexpr int STATE_MACHINE_ITERATION_COUNT = XYZ_AXIS_COUNT * static_cast<int>(STEP_COUNT);
    _filterLooptimeSeconds = STATE_MACHINE_ITERATION_COUNT * _looptimeSeconds;

    const float nyquistFrequencyHz = looprateHz / 2.0F;
    _minHz = config.dyn_notch_min_hz;
    _maxHz = std::fmaxf(_minHz, config.dyn_notch_max_hz);
    _maxHz = std::fminf(_maxHz, nyquistFrequencyHz); // Ensure to not go above the nyquist limit

    _q = static_cast<float>(config.dyn_notch_q) / 100.0F;
    _notchCount = config.dyn_notch_count;

    _sampleCount = std::max(static_cast<size_t>(1), static_cast<size_t>(nyquistFrequencyHz / _maxHz)); // maxHz = 600 & looprateHz = 8000 -> sampleCount = 6
    _sampleCountReciprocal = 1.0F / static_cast<float>(_sampleCount);

    _sampleRateHz = looprateHz / static_cast<float>(_sampleCount);

    _binResolutionHz = _sampleRateHz / static_cast<float>(SDFT_SAMPLE_COUNT); // 18.5Hz per bin at 8kHz looptime and 600Hz maxHz
    _startBin = static_cast<size_t>(lrintf(_minHz / _binResolutionHz));
    if (_startBin == 0) {
        _startBin = 1; // cannot use bin 0 because it is for DC component
    }

    _endBin = static_cast<size_t>(lrintf(_maxHz / _binResolutionHz));
    if (_endBin > SDFT_BIN_COUNT - 1) {
        _endBin = SDFT_BIN_COUNT - 1; // cannot use more than SDFT_BIN_COUNT bins
    }

    for (size_t axis = 0; axis < XYZ_AXIS_COUNT; ++axis) {
        _sdft[axis].init(_startBin, _endBin, _sampleCount);
        for (size_t notchIndex = 0; notchIndex < _notchCount; ++notchIndex) {
            // any init value is fine, but evenly spreading center frequencies across frequency range makes notches stick to peaks quicker
            _centerFrequencyHz[axis][notchIndex] =  _minHz + (static_cast<float>(notchIndex) + 0.5F) * (_maxHz - _minHz) / static_cast<float>(_notchCount);
            _notchFilters[axis][notchIndex].initNotch(_centerFrequencyHz[axis][notchIndex], _looptimeSeconds, _q);
        }
    }
}

/*
Collect gyro data, to be analyzed in update() function

Date is averaged over sampleCount values before sending to SDFT.

sampleCount depends on looprate and maxHz.

For 8kHz looprate, Nyquist frequency is 4kHz,
so for maxHz = 600, sampleCount is int(4000/600) = 6.
*/
void DynamicNotchFilter::push(const xyz_t& sample)
{
    _sampleSum += sample;
    if (_sampleIndex == _sampleCount) {
        _sampleIndex = 0;
        // calculate mean value of accumulated samples
        _sampleAverage = _sampleSum * _sampleCountReciprocal;
        _sampleSum = { 0.0F, 0.0F, 0.0F };
    }

    // SDFT processing in batches to synchronize with incoming downsampled data
    _sdft[X].push(_sampleAverage.x, _sampleIndex);
    _sdft[Y].push(_sampleAverage.y, _sampleIndex);
    _sdft[Z].push(_sampleAverage.z, _sampleIndex);
    ++_sampleIndex;
}

/*!
Find frequency peaks and update filters
Calculation of filters frequencies takes 4 iterations per axis,
so each filter has its frequency set every 12 calls of this function

At 8kHz AHRS loop rate this gives 8kHz/12 = 666Hz, that is frequency update every 1.5ms.
At 4kHz AHRS loop rate this gives 4kHz/12 = 333Hz, that is frequency update every 3ms.
*/
void DynamicNotchFilter::updateNotchFrequencies() // NOLINT(readability-function-cognitive-complexity)
{
    const uint32_t startTimeUs = (_debug.getMode() == DEBUG_FFT_TIME) ? timeUs() : 0;

    _debug.set(DEBUG_FFT_TIME, 0, _state.step);

    // run an iteration of the state machine
    switch (_state.step) {
    case STEP_WINDOW:
        _sdft[_state.axis].calculateWindowSquared(&_sdftData[0]); // calculate power spectral density and put it in _sdftData[]
        // Get total vibrational power in dyn notch range for noise floor estimate in STEP_CALC_FREQUENCIES
        _noiseThreshold = 0.0F;
        for (size_t bin = _startBin; bin <= _endBin; ++bin) {
            _noiseThreshold += _sdftData[bin];
        }
        _debug.set(DEBUG_FFT_TIME, 1, static_cast<int16_t>(timeUs() - startTimeUs));
        break;
    case STEP_DETECT_PEAKS:
        // Get memory ready for new peak data on current axis
        _peaks.fill({0, 0.0F});
        // Search for N biggest peaks in frequency spectrum
        for (size_t bin = _startBin + 1; bin < _endBin; ++bin) {
            // Check if bin is peak
            if ((_sdftData[bin] > _sdftData[bin - 1]) && (_sdftData[bin] > _sdftData[bin + 1])) {
                // Check if peak is big enough to be one of_notchCount biggest peaks.
                // If so, insert into _peaks keeping _peaks sorted in descending VALUE order
                for (size_t notchIndex = 0; notchIndex < _notchCount; ++notchIndex) {
                    if (_sdftData[bin] > _peaks[notchIndex].value) {
                        for (size_t k = _notchCount - 1; k > notchIndex; --k) {
                            _peaks[k] = _peaks[k - 1];
                        }
                        _peaks[notchIndex] = {.bin=bin, .value=_sdftData[bin]};
                        break;
                    }
                }
                ++bin; // if bin is peak, next bin can't be peak so skip
            }
        }
        // Sort peaks into ascending BIN order (eg: 3, 8, 25, 0, 0, ..., 0) so we can later use interpolation between bins
        for (size_t notchIndex = _notchCount - 1; notchIndex > 0; --notchIndex) {
            for (size_t k = 0; k < notchIndex; ++k) {
                // Swap peaks but ignore swapping empty peaks (bin = 0). This leaves
                // empty peaks at the end of peaks array without moving them
                if (_peaks[k].bin > _peaks[k + 1].bin && _peaks[k + 1].bin != 0) {
                    const peak_t temp = _peaks[k];
                    _peaks[k] = _peaks[k + 1];
                    _peaks[k + 1] = temp;
                }
            }
        }
        _debug.set(DEBUG_FFT_TIME, 1, static_cast<int16_t>(timeUs() - startTimeUs));
        break;
    case STEP_CALCULATE_FREQUENCIES:
        // calculate _noiseThreshold (= average power spectral density in dynamic notch range, excluding peaks)
        // _noiseThreshold was initialized in STEP_WINDOW
        {
            size_t peakCount = 0;
            for (size_t notchIndex = 0; notchIndex < _notchCount; ++notchIndex) {
                const size_t peakBin = _peaks[notchIndex].bin;
                if (peakBin != 0) {
                    _noiseThreshold -= _sdftData[peakBin] + 0.75F*(_sdftData[peakBin - 1] + _sdftData[peakBin + 1]);
                    ++peakCount;
                }
            }
            _noiseThreshold /= static_cast<float>(_endBin - _startBin - peakCount + 1);
            // A noise threshold twice the noise floor prevents peak tracking being too sensitive to noise
            _noiseThreshold *= 2.0F;
        }
        for (size_t notchIndex = 0; notchIndex < _notchCount; ++notchIndex) {
            // Only update _centerFrequencyHz if there is a non-void(ie bin!=0) peak and peak value is above noise threshold
            const auto& peak = _peaks[notchIndex];
            if (peak.bin != 0 && peak.value > _noiseThreshold) {
                auto bin = static_cast<float>(peak.bin);
                // get heights of peak bin (y1) and shoulder bins (y0, y2)
                const float y0 = _sdftData[peak.bin - 1];
                const float y1 = _sdftData[peak.bin];
                const float y2 = _sdftData[peak.bin + 1];
                // interpolate to estimate peak position (fit parabola y(x) over y0, y1 and y2, solve dy/dx=0 for x)
                const float denominator = 2.0F*(y0 - 2.0F*y1 + y2);
                if (denominator != 0.0F) {
                    bin += (y0 - y2) / denominator;
                }
                // Convert bin to frequency: frequency = bin * binResolution (bin 0 is 0Hz)
                // resolution is 18.5Hz per bin at 8kHz looptime and 600Hz maxHz
                const float centerFrequencyHz = std::clamp(bin*_binResolutionHz, _minHz, _maxHz);
                // PowerTransfer1 style smoothing moves notch center frequencies rapidly towards big peaks and slowly away, up to 10x faster
                const float cutoffMultiplier = std::clamp(peak.value / _noiseThreshold, 1.0F, 10.0F);
                if (_config.dyn_notch_smoothing) {
                    static constexpr float DYN_NOTCH_SMOOTH_HZ = 4.0F;
                    // calculate center frequency as filtered value of new and old values
                    const float gain = PowerTransferFilter1::gainFromFrequency(DYN_NOTCH_SMOOTH_HZ * cutoffMultiplier, _filterLooptimeSeconds);
                    //Equivalently
                    //const float omega = 2.0F * PI_F*DYN_NOTCH_SMOOTH_HZ * cutoffMultiplier * _filterLooptimeSeconds;
                    //const float gain =  omega/(omega + 1.0F);

                    // Finally update _centerFrequencyHz with smoothing , this is effectively `state += gain*(input - state)`
                    _centerFrequencyHz[_state.axis][notchIndex] += gain * (centerFrequencyHz - _centerFrequencyHz[_state.axis][notchIndex]);
                } else {
                    _centerFrequencyHz[_state.axis][notchIndex] = centerFrequencyHz;
                }
            }
        }
        // calculate maxCenterFrequency for instrumentation (eg display on OSD)
        if (_throttleAbsolute > DYN_NOTCH_MIN_THROTTLE) {
            for (size_t notchIndex = 0; notchIndex < _notchCount; ++notchIndex) {
                _maxCenterFrequencyHz = std::fmaxf(_maxCenterFrequencyHz, _centerFrequencyHz[_state.axis][notchIndex]);
            }
        }
        _debug.set(DEBUG_FFT_TIME, 1, static_cast<int16_t>(timeUs() - startTimeUs));
        break;
    case STEP_UPDATE_FILTERS:
        for (size_t notchIndex = 0; notchIndex < _notchCount; ++notchIndex) {
            // Only update notch filter coefficients if the corresponding peak got its center frequency updated in the previous step
            if (_peaks[notchIndex].bin != 0 && _peaks[notchIndex].value > _noiseThreshold) {
                // setNotchFrequency is a reasonably expensive function involving calculation of sin and cos
                _notchFilters[_state.axis][notchIndex].setNotchFrequency(_centerFrequencyHz[_state.axis][notchIndex]);
            }
        }
        // we have done all 4 steps for the current axis, so advance to the next axis
        if (_state.axis == AXIS_LAST) { _state.axis = AXIS_FIRST; } else { ++_state.axis; }

        _debug.set(DEBUG_FFT_TIME, 1, static_cast<int16_t>(timeUs() - startTimeUs));
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
    for (size_t notchIndex = 0; notchIndex < _notchCount; ++notchIndex) {
        value.x = _notchFilters[X][notchIndex].filter(value.x);
        value.y = _notchFilters[Y][notchIndex].filter(value.y);
        value.z = _notchFilters[Z][notchIndex].filter(value.z);
    }
}
// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
