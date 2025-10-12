#include "Main.h"

#include <AHRS.h>
#include <FlightController.h>
#include <MotorMixerQuadX_DShot.h>
#include <MotorMixerQuadX_DShotBitbang.h>
#include <MotorMixerQuadX_PWM.h>
#include <NonVolatileStorage.h>


FlightController& Main::createFlightController(AHRS& ahrs, IMU_Filters& imuFilters, Debug& debug, NonVolatileStorage& nvs, uint8_t currentPID_Profile, uint8_t mixerType)
{
    (void)mixerType;

    // Statically allocate the MotorMixer object as defined by the build flags.
#if defined(USE_MOTOR_MIXER_QUAD_X_PWM)
    static MotorMixerQuadX_PWM motorMixer(debug, MotorMixerQuadBase::MOTOR_PINS); // NOLINT(misc-const-correctness)
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
    motorMixer.setDynamicIdlerControllerConfig(nvs.loadDynamicIdleControllerConfig(currentPID_Profile));
#else
    motorMixer.setMotorOutputMin(0.055F); // 5.5%
#endif
#elif defined(USE_MOTOR_MIXER_NULL)
    static MotorMixerBase motorMixer(MotorMixerBase::motorCount(static_cast<MotorMixerBase::type_e>(mixerType)), debug);
#else
    static_assert(false && "MotorMixer not specified");
#endif // USE_MOTOR_MIXER

    // Statically allocate the flightController.
    static FlightController flightController(FC_TASK_DENOMINATOR, ahrs, motorMixer, debug);
    ahrs.setVehicleController(&flightController);
    loadPID_ProfileFromNonVolatileStorage(nvs, flightController, currentPID_Profile);

    return flightController;
}