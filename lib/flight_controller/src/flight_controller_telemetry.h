#pragma once

#include <array>
#include <cstdint>

#include <pid_controller.h>

struct flight_controller_quadcopter_telemetry_t {
    pid_error_t roll_rate_error {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    pid_error_t pitch_rate_error {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    pid_error_t yaw_rate_error {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    pid_error_t roll_angle_error {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update
    pid_error_t pitch_angle_error {0, 0, 0, 0, 0}; //!< PID errors calculated in pitch PID update

    enum { MOTOR_COUNT = 4 };
    struct power_rpm_t {
        float power;
        int32_t rpm;
    };
    std::array<power_rpm_t, MOTOR_COUNT> motors;
};
