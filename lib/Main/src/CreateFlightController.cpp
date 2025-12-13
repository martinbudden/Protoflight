#include "FlightController.h"
#include "Main.h"
#include "NonVolatileStorage.h"

#include <AHRS.h>
#include <AHRS_MessageQueue.h>
#include <MotorMixerQuadX_DShot.h>
#include <MotorMixerQuadX_DShotBitbang.h>
#include <MotorMixerQuadX_PWM.h>


FlightController& Main::createFlightController(float taskIntervalSeconds, [[maybe_unused]] Debug& debug, const NonVolatileStorage& nvs)
{
    static AHRS_MessageQueue ahrsMessageQueue;

    const uint32_t outputToMotorsDenominator = OUTPUT_TO_MOTORS_DENOMINATOR;

    const auto taskIntervalMicroseconds = static_cast<uint32_t>(taskIntervalSeconds*1000000.0F);

    // The motor mixer will exist for the duration of the program and so never needs to be deleted, so it is OK to leave its pointer dangling.
    MotorMixerBase* motorMixerPtr = nullptr;


    // Statically allocate the MotorMixer object as defined by the build flags.
#if defined(USE_MOTOR_MIXER_QUAD_X_PWM)

    static MotorMixerQuadX_PWM motorMixer(MotorMixerQuadBase::MOTOR_PINS, &debug);
    motorMixerPtr = &motorMixer;

#elif defined(USE_MOTOR_MIXER_QUAD_X_DSHOT)

#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    static MotorMixerQuadX_DShotBitbang motorMixer(taskIntervalMicroseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS, debug);
#else
    static MotorMixerQuadX_DShot motorMixer(taskIntervalMicroseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS, debug);
#endif
#if defined(USE_DYNAMIC_IDLE)
    motorMixer.setMotorOutputMin(0.0F);
    motorMixer.setDynamicIdlerControllerConfig(nvs.loadDynamicIdleControllerConfig(nvs.getCurrentPidProfileIndex()));
#else
    motorMixer.setMotorOutputMin(0.055F); // 5.5%
#endif
    motorMixerPtr = &motorMixer;

#elif defined(USE_MOTOR_MIXER_NULL)

    const MotorMixerBase::config_t& motorMixerConfig = nvs.loadMotorMixerConfig();
    const auto motorMixerType = static_cast<MotorMixerBase::type_e>(motorMixerConfig.type);
    static MotorMixerBase motorMixer(MotorMixerBase::motorCount(motorMixerType), debug);
    motorMixerPtr = &motorMixer;

#else
    const MotorMixerBase::mixer_config_t& motorMixerConfig = nvs.loadMotorMixerConfig();
    const auto motorMixerType = static_cast<MotorMixerBase::type_e>(motorMixerConfig.type);
    const MotorMixerBase::motor_config_t& motorConfig = nvs.loadMotorConfig();
    const auto motorProtocol = static_cast<MotorMixerBase::motor_protocol_e>(motorConfig.device.motorProtocol);

    switch (motorMixerType) { // NOLINT(hicpp-multiway-paths-covered) switch could be better written as an if/else statement
    case MotorMixerBase::QUAD_X:
        if (motorProtocol == MotorMixerBase::MOTOR_PROTOCOL_PWM || motorProtocol == MotorMixerBase::MOTOR_PROTOCOL_BRUSHED) {
            motorMixerPtr = new MotorMixerQuadX_PWM(MotorMixerQuadBase::MOTOR_PINS, &debug);
        } else {
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
            motorMixerPtr = new MotorMixerQuadX_DShotBitbang(taskIntervalMicroseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS, debug);
#else
            motorMixerPtr = new MotorMixerQuadX_DShot(taskIntervalMicroseconds, outputToMotorsDenominator, MotorMixerQuadBase::MOTOR_PINS, debug);
#endif
        }
        break;
    default:
        assert(false && "MotorMixer type not supported");
    }
    assert((motorMixerPtr != nullptr) && "MotorMixer could not be created");


#endif // USE_MOTOR_MIXER_*

    motorMixerPtr->setMotorConfig(nvs.loadMotorConfig());

    // Statically allocate the flightController.
    static FlightController flightController(taskIntervalMicroseconds, outputToMotorsDenominator, *motorMixerPtr, ahrsMessageQueue, debug);
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

    flightController.setSimplifiedPID_Settings(nvs.loadSimplifiedPID_settings(pidProfile));

    for (uint8_t ii = FlightController::PID_BEGIN; ii < FlightController::PID_COUNT; ++ii) {
        const VehicleControllerBase::PIDF_uint16_t pid16 = nvs.loadPID(ii, pidProfile);
        flightController.setPID_Constants(static_cast<FlightController::pid_index_e>(ii), pid16);
        const std::string pidName = flightController.getPID_Name(static_cast<FlightController::pid_index_e>(ii));
        const PIDF pid = flightController.getPID(static_cast<FlightController::pid_index_e>(ii));
#if !defined(FRAMEWORK_STM32_CUBE)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** %15s PID loaded from NVS: p:%6d, i:%6d, d:%6d, s:%6d, k:%6d\r\n", pidName.c_str(), pid16.kp, pid16.ki, pid16.kd, pid16.ks, pid16.kk);
        print(&buf[0]);
        sprintf(&buf[0], "     %15s                      p:%6.3f, i:%6.3f, d:%6.3f, s:%6.3f, k:%6.3f\r\n", "", pid.getP(), pid.getI(), pid.getD(), pid.getS(), pid.getK());
        print(&buf[0]);
#else
        (void)pidName;
#endif
    }
}
