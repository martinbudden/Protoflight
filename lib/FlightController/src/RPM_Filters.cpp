#include "FastMath.h"
#include "RPM_Filters.h"


void RPM_Filters::init(uint32_t harmonicToUse, float Q)
{
    _harmonicToUse = harmonicToUse;
    _Q = Q;

    // just under  Nyquist frequency (ie just under half sampling rate)
    // for 8kHz loop this is 3840Hz
    _maxFrequencyHz = 480000.0F / static_cast<float>(_looptimeSeconds);
    _halfOfMaxFrequencyHz = _maxFrequencyHz / 2.0F;
    _thirdOfMaxFrequencyHz = _maxFrequencyHz / 3.0F;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    for (size_t motorIndex = 0; motorIndex < _motorCount; ++motorIndex) {
        _filters[motorIndex][FUNDAMENTAL].initNotch(_minFrequencyHz, _looptimeSeconds, _Q);
    }
    if (_harmonicToUse == USE_FUNDAMENTAL_ONLY) {
        return;
    }
    const float minHarmonicFrequency = (_harmonicToUse == USE_FUNDAMENTAL_AND_SECOND_HARMONIC) ? 2.0F * _minFrequencyHz : 3.0F * _minFrequencyHz;
    for (size_t motorIndex = 0; motorIndex < _motorCount; ++motorIndex) {
        _filters[motorIndex][HARMONIC].initNotch(minHarmonicFrequency, _looptimeSeconds, _Q);
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}

/*!
This is called from withing AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.
*/
void RPM_Filters::setFrequencyHz(size_t motorIndex, float frequencyHz)
{
    const float frequencyHzUnclipped = frequencyHz;
    frequencyHz = clip(frequencyHz, _minFrequencyHz, _maxFrequencyHz);

    const float marginFrequencyHz = frequencyHz - _minFrequencyHz;
    const float weightMultiplier = (marginFrequencyHz < _fadeRangeHz) ? marginFrequencyHz / _fadeRangeHz : 1.0F;

    BiquadFilterT<xyz_t>& rpmFilter = _filters[motorIndex][FUNDAMENTAL]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    const float omega = rpmFilter.calculateOmega(frequencyHz);
    // omega = frequency * _2PiLoopTimeSeconds
    // maxFrequency < 0.5 / looptimeSeconds
    // maxOmega = (0.5 / looptimeSeconds) * 2PiLooptimeSeconds = 0.5 * 2PI = PI;
    // so omega is in range [0, PI]
    float s;
    float c;
    FastMath::sincos(omega, s, c);
    const float sinOmega = s;
    const float two_cosOmega = 2.0F * c;
    float weight = _weights[FUNDAMENTAL]*weightMultiplier;

    LOCK_FILTERS();
    rpmFilter.setNotchFrequencyWeighted(sinOmega, two_cosOmega, weight);
    UNLOCK_FILTERS();

    if (_harmonicToUse == USE_FUNDAMENTAL_ONLY) {
        _filterHarmonic &= ~(1U << motorIndex);
        return;
    }

    rpmFilter = _filters[motorIndex][HARMONIC]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    if (_harmonicToUse == USE_FUNDAMENTAL_AND_SECOND_HARMONIC) {
        if (frequencyHzUnclipped > _halfOfMaxFrequencyHz) { // ie 2.0F * frequencyHzUnclipped > _maxFrequencyHz
            // no point filtering the second harmonic if it is above the Nyquist frequency
            _filterHarmonic &= ~(1U << motorIndex);
            return;
        }
        _filterHarmonic |= (1U << motorIndex);
        // sin(2θ) = 2 * sin(θ) * cos(θ)
        // cos(2θ) = 2 * cos^2(θ) - 1
        const float sin_2Omega = sinOmega * two_cosOmega;
        const float two_cos_2Omega = two_cosOmega * two_cosOmega - 2.0F;
        weight = _weights[HARMONIC]*weightMultiplier;
        LOCK_FILTERS();
        rpmFilter.setNotchFrequencyWeighted(sin_2Omega, two_cos_2Omega, weight);
        UNLOCK_FILTERS();
        return;
    }

    // use fundamental and third harmonic
    if (frequencyHzUnclipped > _thirdOfMaxFrequencyHz) { // ie 3.0F * frequencyHzUnclipped > _maxFrequencyHz
        // no point filtering the third harmonic if it is above the Nyquist frequency
        _filterHarmonic &= ~(1U << motorIndex);
        return;
    }
    _filterHarmonic |= (1U << motorIndex);
    // sin(3θ) = 3 * sin(θ)   - 4 * sin^3(θ)
    //         = sin(θ) * ( 3 - 4 * sin^2(θ) )
    //         = sin(θ) * ( 3 - 4 * (1 - cos^2(θ)) )
    //         = sin(θ) * ( 4 * cos^2(θ) - 1)
    // cos(3θ) = 4 * cos^3(θ) - 3 * cos(θ)
    //         = cos(θ) * ( 4 * cos^2(θ) - 3 )
    const float four_cosSquaredOmega = two_cosOmega * two_cosOmega;
    const float sin_3Omega = sinOmega * (four_cosSquaredOmega - 1.0F);
    const float two_cos_3Omega = two_cosOmega * (four_cosSquaredOmega - 3.0F);
    weight = _weights[HARMONIC]*weightMultiplier;
    LOCK_FILTERS();
    rpmFilter.setNotchFrequencyWeighted(sin_3Omega, two_cos_3Omega, weight);
    UNLOCK_FILTERS();
}

/*!
This is called from withing AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.
*/
void RPM_Filters::filter(xyz_t& input, size_t motorIndex) // NOLINT(readability-make-member-function-const) false positive
{
    input = _filters[motorIndex][FUNDAMENTAL].filterWeighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    if (_filterHarmonic & (1U << motorIndex)) {
        input = _filters[motorIndex][HARMONIC].filterWeighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    };
}
