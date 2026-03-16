#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>

typedef std::complex<float> complex_float_t;

template <size_t N>
constexpr std::array<complex_float_t, N> sdft_generate_twiddles_array(float R) {
    std::array<complex_float_t, N> twiddles = {};
    const float m = 3.14159265358979323846F / static_cast<float>(N);
    for (size_t ii = 0; ii < N; ++ii) {
        const float phi = m*static_cast<float>(ii);
        twiddles[ii] = complex_float_t(R*cosf(phi), R*sinf(phi));
    }
    return twiddles;
}

/*!
Sliding Discreet Fourier Transform
*/
template <size_t N>
class Sdft {
public:
    static constexpr size_t SAMPLE_COUNT = N;
    static constexpr size_t BIN_COUNT = N/2;
public:
    Sdft();
    void init(size_t start_bin, size_t end_bin, size_t batch_count);
    void push(float sample, size_t batch_index);
    void calculate_window_squared(float* output);
    void calculate_window(float* output);
    void calculate_magnitude_squared(float* output);
    void calculate_magnitude(float* output);
// for testing
#if defined(FRAMEWORK_TEST)
    size_t get_batch_size() const { return _batch_size; }
    size_t get_batch_count() const { return _batch_count_minus_one + 1; }
    size_t get_start_bin() const { return _start_bin; }
    size_t get_end_bin() const { return _end_bin; }
    size_t get_index() const { return _index; }
    complex_float_t get_twiddle(size_t index) const { return _twiddles[index]; }
#endif
private:
    static constexpr float R = 0.9999F;  // damping factor for Sdft stability (R < 1.0F)
    static constexpr float _rPowerN = powf(R, SAMPLE_COUNT);
    size_t _start_bin {};
    size_t _end_bin {};
    size_t _batch_size {};
    size_t _batch_count_minus_one {};
    size_t _index {}; //!< circular buffer index
    std::array<float, N> _samples {}; //!< circular buffer of samples
    std::array<complex_float_t, BIN_COUNT> _data {};
    const std::array<complex_float_t, BIN_COUNT> _twiddles;
};

template<size_t N>
Sdft<N>::Sdft()
    : _twiddles(sdft_generate_twiddles_array<BIN_COUNT>(R))
{
}

template<size_t N>
void Sdft<N>::init(size_t start_bin, size_t end_bin, size_t batch_count)
{
    _index = 0;

    _start_bin = std::clamp(start_bin, static_cast<size_t>(0), BIN_COUNT - 1);
    _end_bin = std::clamp(end_bin, _start_bin, BIN_COUNT - 1);

    batch_count = (batch_count == 0) ? 1 : batch_count;
    _batch_size = (_end_bin - _start_bin + 1) / batch_count;
    _batch_count_minus_one = batch_count - 1;

    _samples.fill(0.0F);
    _data.fill(0.0F);
}

/*!
Add new sample to frequency spectrum
*/
template<size_t N>
void Sdft<N>::push(float sample, size_t batch_index)
{
    const size_t batch_start = _batch_size*batch_index + _start_bin;
    size_t batch_end = batch_start;

    const float delta = sample - _rPowerN*_samples[_index];

    if (batch_index == _batch_count_minus_one) {
        _samples[_index++] = sample;
        if (_index == SAMPLE_COUNT) {
            _index = 0;
        }
        batch_end += _end_bin - batch_start + 1;
    } else {
        batch_end += _batch_size;
    }

    for (size_t ii = batch_start; ii < batch_end; ++ii) {
        _data[ii] = _twiddles[ii]*(_data[ii] + delta);
    }

    // Ensure proper windowing at the edges of active range
    if (batch_index == 0 && _start_bin > 0 ) {
        const size_t index = _start_bin - 1;
        _data[index] = _twiddles[index]*(_data[index] + delta);
    } else if (batch_index == _batch_count_minus_one && _end_bin < BIN_COUNT - 1) {
        const size_t index = _end_bin + 1;
        _data[index] = _twiddles[index]*(_data[index] + delta);
    }
}

/*!
Get squared magnitude of frequency spectrum with Hanning window applied
Hanning window in frequency domain: X[k] = -0.25*X[k-1] + 0.5*X[k] - 0.25*X[k+1]
*/
template<size_t N>
void Sdft<N>::calculate_window_squared(float* output)
{
    // Apply window at the lower edge of active range
    complex_float_t value = _data[_start_bin];
    if (_start_bin == 0) {
        value -= _data[1];
    } else {
        value -= 0.5F*(_data[_start_bin - 1] + _data[_start_bin + 1]);
    }
    output[_start_bin] = std::norm(value);

    for (size_t ii = _start_bin + 1; ii < _end_bin; ++ii) {
        value = _data[ii] - 0.5F*(_data[ii - 1] + _data[ii + 1]);
        output[ii] = std::norm(value);
    }

    // Apply window at the upper edge of active range
    value = _data[_end_bin];
    if (_end_bin == BIN_COUNT - 1) {
        value -= _data[BIN_COUNT - 2];
    } else {
        value -= 0.5F*(_data[_end_bin - 1] + _data[_end_bin + 1]);
    }
    output[_end_bin] = std::norm(value);
}

/*!
Get magnitude of frequency spectrum with Hanning window applied
*/
template<size_t N>
void Sdft<N>::calculate_window(float* output)
{
    calculate_window_squared(output);
    for (size_t ii = _start_bin; ii <= _end_bin; ++ii) {
        output[ii] = sqrtf(output[ii]);
    }
}

template<size_t N>
void Sdft<N>::calculate_magnitude_squared(float* output)
{
    for (size_t ii = _start_bin; ii <= _end_bin; ++ii) {
        output[ii] = std::norm(_data[ii]);
    }
}

template<size_t N>
void Sdft<N>::calculate_magnitude(float* output)
{
    for (size_t ii = _start_bin; ii <= _end_bin; ++ii) {
        output[ii] = std::abs(_data[ii]);
    }
}
