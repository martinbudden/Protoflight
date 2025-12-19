#pragma once

class AltitudeKalmanFilter {
public:
    struct state_t {
        float velocity;
        float altitude;
        float bias;
    };
    virtual ~AltitudeKalmanFilter() = default;
    AltitudeKalmanFilter() = default;
private:
    // AltitudeKalmanFilter is not copyable or moveable
    AltitudeKalmanFilter(const AltitudeKalmanFilter&) = delete;
    AltitudeKalmanFilter& operator=(const AltitudeKalmanFilter&) = delete;
    AltitudeKalmanFilter(AltitudeKalmanFilter&&) = delete;
    AltitudeKalmanFilter& operator=(AltitudeKalmanFilter&&) = delete;
public:
    AltitudeKalmanFilter::state_t updateState(float altitudeMeasurement, float accelerationMeasurement, float dT);
    void setVelocity(float v);
    void reset();
private:
    state_t _predicted {};
    state_t _estimated {};

    // beta, bias
    const float _beta = -0.01F;

    // estimated P
    float _e11 = 100.0F, _e12 = 0.0F,   _e13 = 0.0F;
    float _e21 = 0.0F,   _e22 = 100.0F, _e23 = 0.0F;
    float _e31 = 0.0F,   _e32 = 0.0F,   _e33 = 100.0F;

    // predicted P
    float _p11 = 0.0F, _p12 = 0.0F, _p13 = 0.0F;
    float _p21 = 0.0F, _p22 = 0.0F, _p23 = 0.0F;
    float _p31 = 0.0F, _p32 = 0.0F, _p33 = 0.0F;

    // Q, process noise covariance matrix
    const float _q1 = 0.1F*0.1F;
    const float _q2 = 1.0F*1.0F;

    // R, measurement covariance matrix
    const float _R = 0.004F*0.004F;
};
