#include "FlightController.h"
#include "Main.h"
#include "NonVolatileStorage.h"

#include <ahrs.h>
#include <motor_mixer_quadx_dshot.h>
#include <motor_mixer_quadx_dshot_bitbang.h>
#include <motor_mixer_quadx_pwm.h>

class AhrsMessageQueue;
class MotorMixerMessageQueue;


MotorMixerBase& Main::createMotorMixer(float taskIntervalSeconds, const NonVolatileStorage& nvs, RpmFilters*& rpmFilters)
{
    rpmFilters = nullptr;

    [[maybe_unused]] const uint32_t outputToMotorsDenominator = OUTPUT_TO_MOTORS_DENOMINATOR;

    [[maybe_unused]] const auto task_interval_microseconds = static_cast<uint32_t>(taskIntervalSeconds*1000000.0F);

    // The motor mixer will exist for the duration of the program and so never needs to be deleted, so it is OK to leave its pointer dangling.
    MotorMixerBase* motorMixerPtr = nullptr;

    // Statically allocate the MotorMixer object as defined by the build flags.
#if defined(USE_MOTOR_MIXER_QUAD_X_PWM)

    static MotorMixerQuadXPwm motorMixer(MotorMixerQuadBase::MOTOR_PINS, outputToMotorsDenominator);
    motorMixerPtr = &motorMixer;

#elif defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)

#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    static MotorMixerQuadXDshotBitbang motorMixer(task_interval_microseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS);
#if defined(USE_RPM_FILTERS)
    static RpmFilters rpm_filters(MotorMixerQuadXDshotBitbang::MOTOR_COUNT, static_cast<float>(task_interval_microseconds) * 0.000001F);
    rpm_filters.set_config(nvs.load_rpm_filters_config());
#endif // USE_RPM_FILTERS
#else
    static MotorMixerQuadXDshot motorMixer(task_interval_microseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS);
#if defined(USE_RPM_FILTERS)
    static RpmFilters rpm_filters(MotorMixerQuadXDshot::MOTOR_COUNT, static_cast<float>(task_interval_microseconds) * 0.000001F);
    rpm_filters.set_config(nvs.load_rpm_filters_config());
#endif // USE_RPM_FILTERS
#endif
#if defined(USE_DYNAMIC_IDLE)
    motorMixer.set_motor_output_min(0.0F);
    motorMixer.set_dynamic_idle_controller_config(nvs.load_dynamic_idle_controller_config(nvs.get_current_pid_profile_index()));
#else
    motorMixer.setMotorOutputMin(0.055F); // 5.5%
#endif
    motorMixerPtr = &motorMixer;

#elif defined(USE_MOTOR_MIXER_NULL)

    const mixer_config_t& motorMixerConfig = nvs.load_motor_mixer_config();
    const auto motorMixerType = static_cast<MotorMixerBase::type_e>(motorMixerConfig.type);
    static MotorMixerBase motorMixer(MotorMixerBase::motorCount(motorMixerType));
    motorMixerPtr = &motorMixer;

#else
    const mixer_config_t& motorMixerConfig = nvs.load_motor_mixer_config();
    const uint8_t motorMixerType = motorMixerConfig.type;
    const motor_config_t& motorConfig = nvs.load_motor_config();
    const uint8_t motorProtocol = motorConfig.device.motor_protocol;

    switch (motorMixerType) { // NOLINT(hicpp-multiway-paths-covered) switch could be better written as an if/else statement
    case MotorMixerBase::QUAD_X:
        if (motorProtocol == MotorMixerBase::MOTOR_PROTOCOL_PWM || motorProtocol == MotorMixerBase::MOTOR_PROTOCOL_BRUSHED) {
            motorMixerPtr = new MotorMixerQuadXPwm(MotorMixerQuadBase::MOTOR_PINS, outputToMotorsDenominator);
        } else {
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
            motorMixerPtr = new MotorMixerQuadXDshotBitbang(task_interval_microseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS);
#else
            motorMixerPtr = new MotorMixerQuadXDshot(task_interval_microseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS);
#endif
        }
        break;
    default:
        assert(false && "MotorMixer type not supported");
    }
    assert((motorMixerPtr != nullptr) && "MotorMixer could not be created");


#endif // USE_MOTOR_MIXER_*

    motorMixerPtr->set_motor_config(nvs.load_motor_config());

    return *motorMixerPtr;
}

FlightController& Main::createFlightController(float taskIntervalSeconds, const NonVolatileStorage& nvs)
{
    const auto task_interval_microseconds = static_cast<uint32_t>(taskIntervalSeconds*1000000.0F);

    // Statically allocate the flightController.
    static FlightController flightController(task_interval_microseconds);
    load_pid_ProfileFromNonVolatileStorage(flightController, nvs, nvs.get_current_pid_profile_index());

    return flightController;
}

/*!
Loads the PID profile for the FlightController. Must be called *after* the FlightController is created.
*/
void Main::load_pid_ProfileFromNonVolatileStorage(FlightController& flightController, const NonVolatileStorage& nvs, uint8_t pidProfile)
{
    flightController.setFiltersConfig(nvs.load_flight_controller_filters_config(pidProfile));
    flightController.setFlightModeConfig(nvs.load_flight_controller_flight_mode_config(pidProfile));
    flightController.setTPA_Config(nvs.load_flight_controller_tpa_config(pidProfile));
    flightController.setAntiGravityConfig(nvs.load_flight_controller_anti_gravity_config(pidProfile));
#if defined(USE_D_MAX)
    flightController.setDMaxConfig(nvs.load_flight_controller_d_max_config(pidProfile));
#endif
#if defined(USE_ITERM_RELAX)
    flightController.setITermRelaxConfig(nvs.load_flight_controller_iterm_relax_config(pidProfile));
#endif
#if defined(USE_YAW_SPIN_RECOVERY)
    flightController.setYawSpinRecoveryConfig(nvs.load_flight_controller_yaw_spin_recovery_config(pidProfile));
#endif
#if defined(USE_CRASH_RECOVERY)
    flightController.setCrashRecoveryConfig(nvs.load_flight_controller_crash_recovery_config(pidProfile));
#endif

    flightController.setSimplifiedPID_Settings(nvs.load_simplified_pid_settings(pidProfile));

    for (uint8_t ii = FlightController::PID_BEGIN; ii < FlightController::PID_COUNT; ++ii) {
        const VehicleControllerBase::PIDF_uint16_t pid16 = nvs.load_pid(ii, pidProfile);
        flightController.set_pid_constants(static_cast<FlightController::pid_index_e>(ii), pid16);
#if !defined(FRAMEWORK_STM32_CUBE)
        const std::string pidName = flightController.getPID_Name(static_cast<FlightController::pid_index_e>(ii));
        const PidController pid = flightController.getPID(static_cast<FlightController::pid_index_e>(ii));
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** %15s PID loaded from NVS: p:%6d, i:%6d, d:%6d, s:%6d, k:%6d\r\n", pidName.c_str(), pid16.kp, pid16.ki, pid16.kd, pid16.ks, pid16.kk);
        print(&buf[0]);
        sprintf(&buf[0], "     %15s                      p:%6.3f, i:%6.3f, d:%6.3f, s:%6.3f, k:%6.3f\r\n", "", static_cast<double>(pid.get_p()), static_cast<double>(pid.get_i()), static_cast<double>(pid.get_d()), static_cast<double>(pid.get_s()), static_cast<double>(pid.get_k()));
        print(&buf[0]);
#endif
    }
}
