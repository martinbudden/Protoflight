#include "AltitudeKalmanFilter.h"
#include "AltitudeTask.h"

#include <AltitudeMessageQueue.h>
#include <ahrs_message_queue.h>
#include <time_microseconds.h>

#include "barometer_base.h"

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
    const uint32_t taskIntervalTicks = _task_interval_microseconds < 1000 ? 1 : pdMS_TO_TICKS(_task_interval_microseconds / 1000);
    _previous_wake_time_ticks = xTaskGetTickCount();
    while (true) {
        // delay until the end of the next taskIntervalTicks
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const BaseType_t was_delayed = xTaskDelayUntil(&_previous_wake_time_ticks, taskIntervalTicks);
            if (was_delayed) {
                _was_delayed = true;
            }
#else
            vTaskDelayUntil(&_previous_wake_time_ticks, taskIntervalTicks);
#endif
        const TickType_t tick_count = xTaskGetTickCount();
        _tick_count_delta = tick_count - _tick_count_previous;
        _tick_count_previous = tick_count;
        const float deltaT = static_cast<float>(_tick_count_delta) * 0.001F;
        if (_tick_count_delta > 0) { // guard against the case of this while loop executing twice on the same tick interval
            _barometer.read_temperature_and_pressure();
            const float altitudeMeasurement = _barometer.calculate_altitude_meters(_barometer.get_pressure_pascals(), _barometer.get_pressure_pascals());

            ahrs_data_t ahrsData {};
            _ahrsMessageQueue.PEEK_AHRS_DATA(ahrsData);
            //!!TODO: calculate vertical component of acceleration corrected for orientation
            const float accelerationMeasurement = ahrsData.acc_gyro_rps.acc.z;
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
