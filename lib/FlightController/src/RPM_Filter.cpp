#include <RPM_Filter.h>
#include <xyz_type.h>

void RPM_Filter::init(uint32_t harmonicToUse, float Q)
{
    _harmonicToUse = harmonicToUse;
    _Q = Q;

    // just under  Nyquist frequency (ie just under half sampling rate)
    // for 8kHz loop this is 3840Hz
    _maxFrequencyHz = 480000.0F / static_cast<float>(_looptimeUs);
    _halfOfMaxFrequencyHz = _maxFrequencyHz / 2.0F;
    _thirdOfMaxFrequencyHz = _maxFrequencyHz / 3.0F;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    for (size_t motorIndex = 0; motorIndex < _motorCount; ++motorIndex) {
        _filters[motorIndex][FUNDAMENTAL][X].initNotch(_minFrequencyHz, _looptimeUs, _Q);
        _filters[motorIndex][FUNDAMENTAL][Y].initNotch(_minFrequencyHz, _looptimeUs, _Q);
        _filters[motorIndex][FUNDAMENTAL][Z].initNotch(_minFrequencyHz, _looptimeUs, _Q);
    }
    if (_harmonicToUse == USE_FUNDAMENTAL_ONLY) {
        return;
    }
    const float minHarmonicFrequency = (_harmonicToUse == USE_FUNDAMENTAL_AND_SECOND_HARMONIC) ? 2.0F * _minFrequencyHz : 3.0F * _minFrequencyHz;
    for (size_t motorIndex = 0; motorIndex < _motorCount; ++motorIndex) {
        _filters[motorIndex][HARMONIC][Y].initNotch(minHarmonicFrequency, _looptimeUs, _Q);
        _filters[motorIndex][HARMONIC][Y].initNotch(minHarmonicFrequency, _looptimeUs, _Q);
        _filters[motorIndex][HARMONIC][Z].initNotch(minHarmonicFrequency, _looptimeUs, _Q);
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}

/*!
This is called from withing AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.
*/
void RPM_Filter::setFrequency(size_t motorIndex, float frequencyHz)
{
    const float frequencyHzUnclipped = frequencyHz;
    frequencyHz = clip(frequencyHz, _minFrequencyHz, _maxFrequencyHz);

    const float marginFrequencyHz = frequencyHz - _minFrequencyHz;
    const float weightMultiplier = (marginFrequencyHz < _fadeRangeHz) ? marginFrequencyHz / _fadeRangeHz : 1.0F;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    BiquadFilter& xFilter = _filters[motorIndex][FUNDAMENTAL][X];

    const float omega = xFilter.calculateOmega(frequencyHz);
    // omega = frequency * _2PiLoopTimeSeconds
    // maxFrequency < 500'000 / looptimeUs = 0.5 / looptimeSeconds
    // maxOmega = (0.5 / looptimeSeconds) * 2PiLooptimeSeconds = 0.5 * 2PI = PI;
    // so omega is in range [0, PI]
    const float sinOmega = sinOrder5(omega);
    const float two_cosOmega = 2.0F * cosOrder5(omega);

    xFilter.setNotchFrequency(sinOmega, two_cosOmega, _weights[FUNDAMENTAL]*weightMultiplier);
    // copy the parameters to the Y and Z filters
    _filters[motorIndex][FUNDAMENTAL][Y].setParameters(xFilter);
    _filters[motorIndex][FUNDAMENTAL][Z].setParameters(xFilter);

    if (_harmonicToUse == USE_FUNDAMENTAL_ONLY) {
        _filterHarmonic &= ~(1U << motorIndex);
        return;
    }
    _filterHarmonic |= (1U << motorIndex);

    xFilter = _filters[motorIndex][HARMONIC][X];

    if (_harmonicToUse == USE_FUNDAMENTAL_AND_SECOND_HARMONIC) {
        if (frequencyHzUnclipped > _halfOfMaxFrequencyHz) { // ie 2.0F * frequencyHzUnclipped > _maxFrequencyHz
            // no point filtering the second harmonic if it is above the Nyquist frequency
            _filterHarmonic &= ~(1U << motorIndex);
            return;
        }
        // sin(2θ) = 2 * sin(θ) * cos(θ)
        // cos(2θ) = 2 * cos^2(θ) - 1
        const float sin_2Omega = sinOmega * two_cosOmega;
        const float two_cos_2Omega = two_cosOmega * two_cosOmega - 2.0F;
        xFilter.setNotchFrequency(sin_2Omega, two_cos_2Omega, _weights[HARMONIC]*weightMultiplier);
    } else {
        // use fundamental and third harmonic
        if (frequencyHzUnclipped > _thirdOfMaxFrequencyHz) { // ie 3.0F * frequencyHzUnclipped > _maxFrequencyHz
            // no point filtering the third harmonic if it is above the Nyquist frequency
            _filterHarmonic &= ~(1U << motorIndex);
            return;
        }
        // sin(3θ) = 3 * sin(θ)   - 4 * sin^3(θ)
        //         = sin(θ) * ( 3 - 4 * sin^2(θ) )
        //         = sin(θ) * ( 3 - 4 * (1 - cos^2(θ)) )
        //         = sin(θ) * ( 4 * cos^2(θ) - 1)
        // cos(3θ) = 4 * cos^3(θ) - 3 * cos(θ)
        //         = cos(θ) * ( 4 * cos^2(θ) - 3 )
        const float four_cosSquaredOmega = two_cosOmega * two_cosOmega;
        const float sin_3Omega = sinOmega * (four_cosSquaredOmega - 1.0F);
        const float two_cos_3Omega = two_cosOmega * (four_cosSquaredOmega - 3.0F);
        xFilter.setNotchFrequency(sin_3Omega, two_cos_3Omega, _weights[HARMONIC]*weightMultiplier);
    }

    // copy the parameters to the Y and Z filters
    _filters[motorIndex][HARMONIC][Y].setParameters(xFilter);
    _filters[motorIndex][HARMONIC][Z].setParameters(xFilter);

    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}

/*!
This is called from withing AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.
*/
void RPM_Filter::filter(xyz_t& input, size_t motorIndex)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    input.x = _filters[motorIndex][FUNDAMENTAL][X].filterWeighted(input.x);
    input.y = _filters[motorIndex][FUNDAMENTAL][Y].filterWeighted(input.y);
    input.z = _filters[motorIndex][FUNDAMENTAL][Z].filterWeighted(input.z);

    if (_filterHarmonic & (1U << motorIndex)) {
        input.x = _filters[motorIndex][HARMONIC][X].filterWeighted(input.x);
        input.y = _filters[motorIndex][HARMONIC][Y].filterWeighted(input.y);
        input.z = _filters[motorIndex][HARMONIC][Z].filterWeighted(input.z);
    };
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}
