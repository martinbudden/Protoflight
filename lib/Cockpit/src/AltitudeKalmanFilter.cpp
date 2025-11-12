#include "AltitudeKalmanFilter.h"


/*
https://thekalmanfilter.com/kalman-filter-explained-simply/

| S | description                     | type                | usage        |
| - | ------------------------------- | ------------------- | ------------ |
| x | state variable                  | n x 1 column vector | Output       |
| P | state covariance matrix         | n x n matrix        | Output       |
| z | measurement                     | m x 1 column vector | Input        |
| A | state transition matrix         | n x n matrix        | System Model |
| H | state-to-measurement matrix     | m x n matrix        | System Model |
| R | measurement covariance matrix   | m x m matrix        | Input        |
| Q | process noise covariance matrix | n x n matrix        | System Model |
| K | Kalman Gain                     | n x m               | Internal     |
*/

AltitudeKalmanFilter::state_t AltitudeKalmanFilter::updateState(float altitudeMeasurement, float accelerationMeasurement, float dT)
{
    // predicted state
    _predicted.velocity = _estimated.velocity + (accelerationMeasurement - _estimated.bias)*dT;
    _predicted.altitude = _estimated.altitude + _estimated.velocity*dT;
    _predicted.bias     = _estimated.bias*(1.0F + _beta*dT);

    // update predicted P
    _p11 = _e11 - dT*((_e31 + _e13) + dT*_e33 + dT*_q1);
    _p12 = dT*((_e11 - _e32) - dT*_e31) + _e12;
    _p13 = (1.0F + _beta*dT)*(_e13 - dT*_e33);

    _p21 = dT*((_e11 - _e23) - dT*_e13) + _e21;
    _p22 = dT*(dT*_e11 + (_e21 + _e12)) + _e22;
    _p23 = (1.0F + _beta*dT)*(_e23 + dT*_e13);

    _p31 = (1.0F + _beta*dT)*(_e31 - dT*_e33);
    _p32 = (1.0F + _beta*dT)*(_e32 + dT*_e31);
    _p33 = (1.0F + _beta*dT)*(1 + _beta*dT)*_e33 + dT*dT*_q2;

    // update Kalman gain
    const float s = _p22 + _R;
    const float k1 = _p12 / s;
    const float k2 = _p22 / s;
    const float k3 = _p32 / s;

    // update estimate
    const float e = altitudeMeasurement - _predicted.altitude;
    _estimated.velocity = _predicted.velocity + k1*e;
    _estimated.altitude = _predicted.altitude + k2*e;
    _estimated.bias     = _predicted.bias + k3*e;

    // update estimated P
    _e11 = _p11 - k1*_p21;
    _e12 = _p12 - k1*_p22;
    _e13 = _p13 - k1*_p23;
    _e21 = _p21*(1.0F - k2);
    _e22 = _p22*(1.0F - k2);
    _e23 = _p23*(1.0F - k2);
    _e31 = -k3*_p21 + _p31;
    _e32 = -k3*_p22 + _p32;
    _e33 = -k3*_p23 + _p33;

    return _estimated;
}

void AltitudeKalmanFilter::setVelocity(float velocity)
{
    _estimated.velocity = velocity;
}

void AltitudeKalmanFilter::reset()
{
    _e11 = 100.0F, _e12 = 0.0F,   _e13 = 0.0F;
    _e21 = 0.0F,   _e22 = 100.0F, _e23 = 0.0F;
    _e31 = 0.0F,   _e32 = 0.0F,   _e33 = 100.0F;
}
