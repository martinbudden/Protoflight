#pragma once

#include <array>
#include <cstdint>

#include <pid_controller.h>

struct flight_controller_quadcopter_telemetry_t {
    pid_error_t rollRateError {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    pid_error_t pitchRateError {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    pid_error_t yawRateError {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    pid_error_t rollAngleError {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    pid_error_t pitchAngleError {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update

    enum { MOTOR_COUNT = 4 };
    struct power_rpm_t {
        float power;
        int32_t rpm;
    };
    std::array<power_rpm_t, MOTOR_COUNT> motors;
};
