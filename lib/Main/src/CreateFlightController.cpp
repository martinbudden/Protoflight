#include "Main.h"

#include <AHRS.h>
#include <FlightController.h>
#include <MotorMixerQuadX_DShot.h>
#include <MotorMixerQuadX_DShotBitbang.h>
#include <MotorMixerQuadX_PWM.h>
#include <NonVolatileStorage.h>


FlightController& Main::createFlightController(AHRS& ahrs, IMU_Filters& imuFilters, Debug& debug, const NonVolatileStorage& nvs)
{
    // Statically allocate the MotorMixer object as defined by the build flags.
#if defined(USE_MOTOR_MIXER_QUAD_X_PWM)
    static MotorMixerQuadX_PWM motorMixer(debug, MotorMixerQuadBase::MOTOR_PINS);
    (void)imuFilters;
#elif defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)
    const uint32_t motorTaskIntervalMicroseconds = ahrs.getTaskIntervalMicroseconds() / FC_TASK_DENOMINATOR;
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    static MotorMixerQuadX_DShotBitbang motorMixer(motorTaskIntervalMicroseconds, debug, MotorMixerQuadBase::MOTOR_PINS, *imuFilters.getRPM_Filters());
#else
    static MotorMixerQuadX_DShot motorMixer(motorTaskIntervalMicroseconds, debug, MotorMixerQuadBase::MOTOR_PINS, *imuFilters.getRPM_Filters());
#endif
#if defined(USE_DYNAMIC_IDLE)
    motorMixer.setMotorOutputMin(0.0F);
    motorMixer.setDynamicIdlerControllerConfig(nvs.loadDynamicIdleControllerConfig(nvs.getCurrentPidProfileIndex()));
#else
    motorMixer.setMotorOutputMin(0.055F); // 5.5%
#endif
#elif defined(USE_MOTOR_MIXER_NULL)
    static MotorMixerBase motorMixer(MotorMixerBase::motorCount(nvs.loadMotorMixerType()), debug);
    (void)imuFilters;
#else
    static_assert(false && "MotorMixer not specified");
#endif // USE_MOTOR_MIXER

    // Statically allocate the flightController.
    static FlightController flightController(FC_TASK_DENOMINATOR, ahrs, motorMixer, debug);
    loadPID_ProfileFromNonVolatileStorage(flightController, nvs, nvs.getCurrentPidProfileIndex());

    return flightController;
}

/*!
Loads the PID profile for the FlightController. Must be called *after* the FlightController is created.
*/
void Main::loadPID_ProfileFromNonVolatileStorage(FlightController& flightController, const NonVolatileStorage& nvs, uint8_t pidProfile)
{
    flightController.setFiltersConfig(nvs.loadFlightControllerFiltersConfig(pidProfile));
    flightController.setFlightModeConfig(nvs.loadFlightControllerFlightModeConfig(pidProfile));
    flightController.setTPA_Config(nvs.loadFlightControllerTPA_Config(pidProfile));
    flightController.setAntiGravityConfig(nvs.loadFlightControllerAntiGravityConfig(pidProfile));
#if defined(USE_D_MAX)
    flightController.setDMaxConfig(nvs.loadFlightControllerDMaxConfig(pidProfile));
#endif
#if defined(USE_ITERM_RELAX)
    flightController.setITermRelaxConfig(nvs.loadFlightControllerITermRelaxConfig(pidProfile));
#endif
#if defined(USE_YAW_SPIN_RECOVERY)
    flightController.setYawSpinRecoveryConfig(nvs.loadFlightControllerYawSpinRecoveryConfig(pidProfile));
#endif
#if defined(USE_CRASH_RECOVERY)
    flightController.setCrashRecoveryConfig(nvs.loadFlightControllerCrashRecoveryConfig(pidProfile));
#endif
    for (int ii = FlightController::PID_BEGIN; ii < FlightController::PID_COUNT; ++ii) {
        const VehicleControllerBase::PIDF_uint16_t pid = nvs.loadPID(ii, pidProfile);
        flightController.setPID_Constants(static_cast<FlightController::pid_index_e>(ii), pid);
        const std::string pidName = flightController.getPID_Name(static_cast<FlightController::pid_index_e>(ii));
#if !defined(FRAMEWORK_STM32_CUBE)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** %15s PID loaded from NVS: p:%d, i:%d, d:%d, s:%d, k:%d\r\n", pidName.c_str(), pid.kp, pid.ki, pid.kd, pid.ks, pid.kk);
        print(&buf[0]);
#else
        (void)pidName;
#endif
    }
}
