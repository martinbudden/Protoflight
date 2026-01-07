#include "AltitudeKalmanFilter.h"
#include "AltitudeTask.h"
#include "BarometerBase.h"

#include <AHRS_MessageQueue.h>
#include <AltitudeMessageQueue.h>
#include <TimeMicroseconds.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <task.h>
#endif
#endif


/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
[[noreturn]] void AltitudeTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    const uint32_t taskIntervalTicks = _taskIntervalMicroseconds < 1000 ? 1 : pdMS_TO_TICKS(_taskIntervalMicroseconds / 1000);
    _previousWakeTimeTicks = xTaskGetTickCount();
    while (true) {
        // delay until the end of the next taskIntervalTicks
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const BaseType_t wasDelayed = xTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);
            if (wasDelayed) {
                _wasDelayed = true;
            }
#else
            vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);
#endif
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const float deltaT = static_cast<float>(_tickCountDelta) * 0.001F;
        if (_tickCountDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
            _barometer.readTemperatureAndPressure();
            const float altitudeMeasurement = _barometer.calculateAltitudeMeters(_barometer.getPressurePascals(), _barometer.getPressurePascals());

            ahrs_data_t ahrsData {};
            _ahrsMessageQueue.PEEK_AHRS_DATA(ahrsData);
            //!!TODO: calculate vertical component of acceleration corrected for orientation
            const float accelerationMeasurement = ahrsData.accGyroRPS.acc.z;
            const AltitudeKalmanFilter::state_t kalmanFilterState =_altitudeKalmanFilter.updateState(altitudeMeasurement, accelerationMeasurement, deltaT);

            const altitude_data_t altitudeData {
                .altitudeMeters = kalmanFilterState.altitude,
                .altitudeVelocityMetersPerSecond = kalmanFilterState.velocity
            };
            _altitudeMessageQueue.SEND_ALTITUDE_DATA(altitudeData);
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for AHRS::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void AltitudeTask::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<AltitudeTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
