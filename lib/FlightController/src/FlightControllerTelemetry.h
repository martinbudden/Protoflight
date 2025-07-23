#pragma once

#include <PIDF.h>
#include <array>
#include <cstdint>

struct flight_controller_quadcopter_telemetry_t {
    PIDF::error_t rollRateError {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    PIDF::error_t pitchRateError {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    PIDF::error_t yawRateError {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    PIDF::error_t rollAngleError {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    PIDF::error_t pitchAngleError {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update

    enum { MOTOR_COUNT = 4 };
    struct power_rpm_t {
        float power;
        int32_t rpm;
    };
    std::array<power_rpm_t, MOTOR_COUNT> motors;
};
