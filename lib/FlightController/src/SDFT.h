#pragma once

#include <array>
#include <complex>
#include <cstddef>

typedef std::complex<float> complex_float_t;

/*!
Sliding Discreet Fourier Transform
*/
template <size_t N>
class SDFT {
public:
    enum { SAMPLE_COUNT = N, BIN_COUNT = N/2 };
public:
    SDFT();
    void init(size_t startBin, size_t endBin, size_t batchCount);
    void push(float sample, size_t batchIndex);
    void calculateWindowSquared(float* output);
    void calculateWindow(float* output);
    void calculateMagnitudeSquared(float* output);
    void calculateMagnitude(float* output);
    static inline size_t clip(size_t value, size_t min, size_t max) { return value < min ? min : value > max ? max : value; }
// for testing
#if defined(FRAMEWORK_TEST)
    size_t getBatchSize() const { return _batchSize; }
    size_t getBatchCount() const { return _batchCountMinusOne + 1; }
    size_t getStartBin() const { return _startBin; }
    size_t getEndBin() const { return _endBin; }
    size_t getIndex() const { return _index; }
#endif
private:
    static constexpr float SDFT_R = 0.9999F;  // damping factor for guaranteed SDFT stability (r < 1.0f)
    float _rPowerN {};  // SDFT_R to the power of SAMPLE_COUNT
    size_t _startBin {};
    size_t _endBin {};
    size_t _batchSize {};
    size_t _batchCountMinusOne {};
    size_t _index {};// circular buffer index
    std::array<float, N> _samples {}; // circular buffer of samples
    std::array<complex_float_t, BIN_COUNT> _data {};
    const std::array<complex_float_t, BIN_COUNT> _twiddles {};
};

template<size_t N>
SDFT<N>::SDFT()
{
    _rPowerN = powf(SDFT_R, SAMPLE_COUNT);
    const float m = 2.0F*static_cast<float>(M_PI) / static_cast<float>(SAMPLE_COUNT);
    for (size_t ii = 0; ii < BIN_COUNT; ++ii) {
        const float phi = m*static_cast<float>(ii);
        const_cast<complex_float_t&>(_twiddles[ii]) = complex_float_t(SDFT_R*cosf(phi), SDFT_R*sinf(phi));
    }
}

template<size_t N>
void SDFT<N>::init(size_t startBin, size_t endBin, size_t batchCount)
{
    _index = 0;

    _startBin = clip(startBin, 0, BIN_COUNT - 1);
    _endBin = clip(endBin, _startBin, BIN_COUNT - 1);

    batchCount = (batchCount == 0) ? 1 : batchCount;
    _batchSize = (_endBin - _startBin + 1) / batchCount;
    _batchCountMinusOne = batchCount - 1;

    _samples.fill(0.0F);
    _data.fill(0.0F);
}

/*!
Add new sample to frequency spectrum
*/
template<size_t N>
void SDFT<N>::push(float sample, size_t batchIndex)
{
    const size_t batchStart = _batchSize*batchIndex + _startBin;
    size_t batchEnd = batchStart;

    const float delta = sample - _rPowerN*_samples[_index];

    if (batchIndex == _batchCountMinusOne) {
        _samples[_index++] = sample;
        if (_index == SAMPLE_COUNT) {
            _index = 0;
        }
        batchEnd += _endBin - batchStart + 1;
    } else {
        batchEnd += _batchSize;
    }

    for (size_t ii = batchStart; ii < batchEnd; ++ii) {
        _data[ii] = _twiddles[ii]*(_data[ii] + delta);
    }

    // Ensure proper windowing at the edges of active range
    if (batchIndex == 0 && _startBin > 0 ) {
        const size_t index = _startBin - 1;
        _data[index] = _twiddles[index]*(_data[index] + delta);
    } else if (batchIndex == _batchCountMinusOne && _endBin < BIN_COUNT - 1) {
        const size_t index = _endBin + 1;
        _data[index] = _twiddles[index]*(_data[index] + delta);
    }
}

/*!
Get squared magnitude of frequency spectrum with Hanning window applied
Hanning window in frequency domain: X[k] = -0.25*X[k-1] + 0.5*X[k] - 0.25*X[k+1]
*/
template<size_t N>
void SDFT<N>::calculateWindowSquared(float* output)
{
    // Apply window at the lower edge of active range
    complex_float_t value = _data[_startBin];
    if (_startBin == 0) {
        value -= _data[1];
    } else {
        value -= 0.5F*(_data[_startBin - 1] + _data[_startBin + 1]);
    }
    output[_startBin] = std::norm(value);

    for (size_t ii = _startBin + 1; ii < _endBin; ++ii) {
        value = _data[ii] - 0.5F*(_data[ii - 1] + _data[ii + 1]);
        output[ii] = std::norm(value);
    }

    // Apply window at the upper edge of active range
    value = _data[_endBin];
    if (_endBin == BIN_COUNT - 1) {
        value -= _data[BIN_COUNT - 2];
    } else {
        value -= 0.5F*(_data[_endBin - 1] + _data[_endBin + 1]);
    }
    output[_endBin] = std::norm(value);
}

/*!
Get magnitude of frequency spectrum with Hanning window applied
*/
template<size_t N>
void SDFT<N>::calculateWindow(float* output)
{
    calculateWindowSquared(output);
    for (size_t ii = _startBin; ii <= _endBin; ++ii) {
        output[ii] = sqrtf(output[ii]);
    }
}

template<size_t N>
void SDFT<N>::calculateMagnitudeSquared(float* output)
{
    for (size_t ii = _startBin; ii <= _endBin; ++ii) {
        output[ii] = std::norm(_data[ii]);
    }
}

template<size_t N>
void SDFT<N>::calculateMagnitude(float* output)
{
    for (size_t ii = _startBin; ii <= _endBin; ++ii) {
        output[ii] = std::abs(_data[ii]);
    }
}
