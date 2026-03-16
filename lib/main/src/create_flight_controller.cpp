#include "flight_controller.h"
#include "main.h"
#include "non_volatile_storage.h"

#include <ahrs.h>
#include <motor_mixer_quadx_dshot.h>
#include <motor_mixer_quadx_dshot_bitbang.h>
#include <motor_mixer_quadx_pwm.h>

class AhrsMessageQueue;
class MotorMixerMessageQueue;


MotorMixerBase& Main::create_motor_mixer(float task_interval_seconds, const NonVolatileStorage& nvs, RpmFilters*& rpm_filters_ptr)
{
    rpm_filters_ptr = nullptr;

    [[maybe_unused]] const uint32_t outputToMotorsDenominator = OUTPUT_TO_MOTORS_DENOMINATOR;

    [[maybe_unused]] const auto task_interval_microseconds = static_cast<uint32_t>(task_interval_seconds*1000000.0F);

    // The motor mixer will exist for the duration of the program and so never needs to be deleted, so it is OK to leave its pointer dangling.
    MotorMixerBase* motor_mixer_ptr = nullptr;

    // Statically allocate the MotorMixer object as defined by the build flags.
#if defined(USE_MOTOR_MIXER_QUAD_X_PWM)

    static MotorMixerQuadXPwm motor_mixer(MotorMixerQuadBase::MOTOR_PINS, outputToMotorsDenominator);
    motor_mixer_ptr = &motor_mixer;

#elif defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)

#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    static MotorMixerQuadXDshotBitbang motor_mixer(task_interval_microseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS);
#if defined(USE_RPM_FILTERS)
    static RpmFilters rpm_filters(MotorMixerQuadXDshotBitbang::MOTOR_COUNT, static_cast<float>(task_interval_microseconds) * 0.000001F);
    rpm_filters_ptr = &rpm_filters;
    rpm_filters.set_config(nvs.load_rpm_filters_config());
#endif // USE_RPM_FILTERS
#else
    static MotorMixerQuadXDshot motor_mixer(task_interval_microseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS);
#if defined(USE_RPM_FILTERS)
    static RpmFilters rpm_filters(MotorMixerQuadXDshot::MOTOR_COUNT, static_cast<float>(task_interval_microseconds) * 0.000001F);
    rpm_filters_ptr = &rpm_filters;
    rpm_filters.set_config(nvs.load_rpm_filters_config());
#endif // USE_RPM_FILTERS
#endif
#if defined(USE_DYNAMIC_IDLE)
    motor_mixer.set_motor_output_min(0.0F);
    motor_mixer.set_dynamic_idle_controller_config(nvs.load_dynamic_idle_controller_config(nvs.get_current_pid_profile_index()));
#else
    motor_mixer.setMotorOutputMin(0.055F); // 5.5%
#endif
    motor_mixer_ptr = &motor_mixer;

#elif defined(USE_MOTOR_MIXER_NULL)

    const mixer_config_t& motor_mixerConfig = nvs.load_motor_mixer_config();
    const auto motor_mixerType = static_cast<MotorMixerBase::type_e>(motor_mixerConfig.type);
    static MotorMixerBase motor_mixer(MotorMixerBase::motorCount(motor_mixerType));
    motor_mixer_ptr = &motor_mixer;

#else
    const mixer_config_t& motor_mixerConfig = nvs.load_motor_mixer_config();
    const uint8_t motor_mixerType = motor_mixerConfig.type;
    const motor_config_t& motorConfig = nvs.load_motor_config();
    const uint8_t motorProtocol = motorConfig.device.motor_protocol;

    switch (motor_mixerType) { // NOLINT(hicpp-multiway-paths-covered) switch could be better written as an if/else statement
    case MotorMixerBase::QUAD_X:
        if (motorProtocol == MotorMixerBase::MOTOR_PROTOCOL_PWM || motorProtocol == MotorMixerBase::MOTOR_PROTOCOL_BRUSHED) {
            motor_mixer_ptr = new MotorMixerQuadXPwm(MotorMixerQuadBase::MOTOR_PINS, outputToMotorsDenominator);
        } else {
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
            motor_mixer_ptr = new MotorMixerQuadXDshotBitbang(task_interval_microseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS);
#else
            motor_mixer_ptr = new MotorMixerQuadXDshot(task_interval_microseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS);
#endif
        }
        break;
    default:
        assert(false && "MotorMixer type not supported");
    }
    assert((motor_mixer_ptr != nullptr) && "MotorMixer could not be created");


#endif // USE_MOTOR_MIXER_*

    motor_mixer_ptr->set_motor_config(nvs.load_motor_config());

    return *motor_mixer_ptr;
}

FlightController& Main::create_flight_controller(float task_interval_seconds, const NonVolatileStorage& nvs)
{
    const auto task_interval_microseconds = static_cast<uint32_t>(task_interval_seconds*1000000.0F);

    // Statically allocate the flight_controller.
    static FlightController flight_controller(task_interval_microseconds);
    load_pid_ProfileFromNonVolatileStorage(flight_controller, nvs, nvs.get_current_pid_profile_index());

    return flight_controller;
}

/*!
Loads the PID profile for the FlightController. Must be called *after* the FlightController is created.
*/
void Main::load_pid_ProfileFromNonVolatileStorage(FlightController& flight_controller, const NonVolatileStorage& nvs, uint8_t pid_profile)
{
    flight_controller.set_filters_config(nvs.load_flight_controller_filters_config(pid_profile));
    flight_controller.set_flight_mode_config(nvs.load_flight_controller_flight_mode_config(pid_profile));
    flight_controller.set_tpa_config(nvs.load_flight_controller_tpa_config(pid_profile));
    flight_controller.set_anti_gravity_config(nvs.load_flight_controller_anti_gravity_config(pid_profile));
#if defined(USE_DMAX)
    flight_controller.set_dmax_config(nvs.load_flight_controller_dmax_config(pid_profile));
#endif
#if defined(USE_ITERM_RELAX)
    flight_controller.set_iterm_relax_config(nvs.load_flight_controller_iterm_relax_config(pid_profile));
#endif
#if defined(USE_YAW_SPIN_RECOVERY)
    flight_controller.set_yaw_spin_recovery_config(nvs.load_flight_controller_yaw_spin_recovery_config(pid_profile));
#endif
#if defined(USE_CRASH_RECOVERY)
    flight_controller.set_crash_recovery_config(nvs.load_flight_controller_crash_recovery_config(pid_profile));
#endif

    flight_controller.set_simplified_pid_Settings(nvs.load_simplified_pid_settings(pid_profile));

    for (uint8_t ii = FlightController::PID_BEGIN; ii < FlightController::PID_COUNT; ++ii) {
        const pid_constants_uint16_t pid16 = nvs.load_pid(ii, pid_profile);
        flight_controller.set_pid_constants(static_cast<FlightController::pid_index_e>(ii), pid16);
#if !defined(FRAMEWORK_STM32_CUBE)
        const std::string pidName = flight_controller.get_pid_name(static_cast<FlightController::pid_index_e>(ii));
        const PidController pid = flight_controller.get_pid(static_cast<FlightController::pid_index_e>(ii));
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** %15s PID loaded from NVS: p:%6d, i:%6d, d:%6d, s:%6d, k:%6d\r\n", pidName.c_str(), pid16.kp, pid16.ki, pid16.kd, pid16.ks, pid16.kk);
        print(&buf[0]);
        sprintf(&buf[0], "     %15s                      p:%6.3f, i:%6.3f, d:%6.3f, s:%6.3f, k:%6.3f\r\n", "", static_cast<double>(pid.get_p()), static_cast<double>(pid.get_i()), static_cast<double>(pid.get_d()), static_cast<double>(pid.get_s()), static_cast<double>(pid.get_k()));
        print(&buf[0]);
#endif
    }
}
