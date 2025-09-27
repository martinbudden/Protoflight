#include "RPM_Filters.h"
#include <FastTrigonometry.h>


void RPM_Filters::setConfig(const config_t& config)
{
    _config = config;
    _Q = static_cast<float>(_config.rpm_filter_q) * 0.01F;

    // just under  Nyquist frequency (ie just under half sampling rate)
    // for 8kHz loop this is 3840Hz
    _maxFrequencyHz = 480000.0F / static_cast<float>(_looptimeSeconds);
    _halfOfMaxFrequencyHz = _maxFrequencyHz / 2.0F;
    _thirdOfMaxFrequencyHz = _maxFrequencyHz / 3.0F;
    _minFrequencyHz = _config.rpm_filter_min_hz;
    _fadeRangeHz = _config.rpm_filter_fade_range_hz;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    for (size_t ii = 0; ii < _config.rpm_filter_harmonics; ++ii) {
        for (size_t motorIndex = 0; motorIndex < _motorCount; ++motorIndex) {
            _filters[motorIndex][ii].initNotch(_minFrequencyHz * static_cast<float>(ii + 1), _looptimeSeconds, _Q);
        }
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)

    if (config.rpm_filter_lpf_hz == 0) {
        for (auto& motorRPM_filter : _motorRPM_Filters) {
            motorRPM_filter.setToPassthrough();
        }
    } else {
        for (auto& motorRPM_filter : _motorRPM_Filters) {
            motorRPM_filter.setCutoffFrequencyAndReset(config.rpm_filter_lpf_hz, _looptimeSeconds);
        }
    }
}

/*!
NOTE: CALLED FROM WITHIN THE FLIGHT CONTROLLER TASK

This is called from withing MotorMixerQuadX_DShot::outputToMotors (ie the main IMU/PID loop) and so needs to be FAST.
*/
void RPM_Filters::setFrequencyHz(size_t motorIndex, float frequencyHz)
{
    frequencyHz = _motorRPM_Filters[motorIndex].filter(frequencyHz);

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
    FastTrigonometry::sincos(omega, s, c);
    const float sinOmega = s;
    const float two_cosOmega = 2.0F * c;
    float weight = _weights[FUNDAMENTAL]*weightMultiplier;

    LOCK_FILTERS();
    rpmFilter.setNotchFrequencyWeighted(sinOmega, two_cosOmega, weight);
    UNLOCK_FILTERS();

    rpmFilter = _filters[motorIndex][SECOND_HARMONIC]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    if (_config.rpm_filter_weights[SECOND_HARMONIC] != 0 && _config.rpm_filter_harmonics >= 2) {
        if (frequencyHzUnclipped > _halfOfMaxFrequencyHz) { // ie 2.0F * frequencyHzUnclipped > _maxFrequencyHz
            // no point filtering the second harmonic if it is above the Nyquist frequency
            _weights[SECOND_HARMONIC] = 0.0F;
            return;
        }
        _weights[SECOND_HARMONIC] = _config.rpm_filter_weights[SECOND_HARMONIC] * 0.01F;
        // sin(2θ) = 2 * sin(θ) * cos(θ)
        // cos(2θ) = 2 * cos^2(θ) - 1
        const float sin_2Omega = sinOmega * two_cosOmega;
        const float two_cos_2Omega = two_cosOmega * two_cosOmega - 2.0F;
        LOCK_FILTERS();
        rpmFilter.setNotchFrequencyWeighted(sin_2Omega, two_cos_2Omega, _weights[SECOND_HARMONIC]*weightMultiplier);
        UNLOCK_FILTERS();
        return;
    }

    if (_config.rpm_filter_weights[THIRD_HARMONIC] != 0 && _config.rpm_filter_harmonics >= 3) {
        if (frequencyHzUnclipped > _thirdOfMaxFrequencyHz) { // ie 3.0F * frequencyHzUnclipped > _maxFrequencyHz
            // no point filtering the third harmonic if it is above the Nyquist frequency
            _weights[THIRD_HARMONIC] = 0.0F;
            return;
        }
        _weights[THIRD_HARMONIC] = _config.rpm_filter_weights[THIRD_HARMONIC] * 0.01F;
        // sin(3θ) = 3 * sin(θ)   - 4 * sin^3(θ)
        //         = sin(θ) * ( 3 - 4 * sin^2(θ) )
        //         = sin(θ) * ( 3 - 4 * (1 - cos^2(θ)) )
        //         = sin(θ) * ( 4 * cos^2(θ) - 1)
        // cos(3θ) = 4 * cos^3(θ) - 3 * cos(θ)
        //         = cos(θ) * ( 4 * cos^2(θ) - 3 )
        const float four_cosSquaredOmega = two_cosOmega * two_cosOmega;
        const float sin_3Omega = sinOmega * (four_cosSquaredOmega - 1.0F);
        const float two_cos_3Omega = two_cosOmega * (four_cosSquaredOmega - 3.0F);
        LOCK_FILTERS();
        rpmFilter.setNotchFrequencyWeighted(sin_3Omega, two_cos_3Omega, _weights[THIRD_HARMONIC]*weightMultiplier);
        UNLOCK_FILTERS();
    }
}

/*!
NOTE: CALLED FROM WITHIN THE AHRS TASK

This is called from withing AHRS::readIMUandUpdateOrientation() (ie the main IMU/PID loop) and so needs to be FAST.
*/
void RPM_Filters::filter(xyz_t& input, size_t motorIndex) // NOLINT(readability-make-member-function-const) false positive
{
    input = _filters[motorIndex][FUNDAMENTAL].filterWeighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    if (_weights[SECOND_HARMONIC] != 0.0F) {
        input = _filters[motorIndex][SECOND_HARMONIC].filterWeighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    };
    if (_weights[THIRD_HARMONIC] != 0.0F) {
        input = _filters[motorIndex][THIRD_HARMONIC].filterWeighted(input); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    };
}
