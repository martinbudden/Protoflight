#include "ahrs_task.h"

#include <ahrs.h>
#include <ahrs_message_queue.h>
#include <imu_filters_base.h>
#include <motor_commands.h>
#include <motor_mixer_message_queue.h>
#include <vehicle_controller_base.h>

#include <imu_base.h>
#include <time_microseconds.h>

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
loop() function
*/
void AhrsTask::loop()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    // record tick_counts for instrumentation. Not sure if this is useful anymore
    const TickType_t tick_count = xTaskGetTickCount();
    _tick_count_delta = tick_count - _tick_count_previous;
    _tick_count_previous = tick_count;
#endif

    const time_us32_t time_microseconds = time_us();
    _time_microseconds_previous = time_microseconds;

    if (time_microseconds > _time_microseconds_previous) { // guard against the case of this while loop executing twice on the same tick interval
        _time_microseconds_delta = time_microseconds - _time_microseconds_previous;
        //const ahrs_data_t& ahrs_data = _context.ahrs.read_imu_and_update_orientation(time_microseconds, _time_microseconds_delta, _context.imu_filters, _context.vehicle_controller, _context.debug);
        acc_gyro_rps_t acc_gyro_rps = _context.ahrs.read_imu();
#if defined(USE_BLACKBOX) || defined(USE_BACKCHANNEL) || defined(USE_DASHBOARD)
        const xyz_t gyro_rps_unfiltered = acc_gyro_rps.gyro_rps;
#endif

        const float delta_t = static_cast<float>(_time_microseconds_delta) * 0.000001F; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        _context.imu_filters.filter(acc_gyro_rps.acc, acc_gyro_rps.gyro_rps, delta_t, _context.debug);
        const Quaternion orientation = _context.ahrs.update_orientation(acc_gyro_rps.acc, acc_gyro_rps.gyro_rps, delta_t, _context.vehicle_controller);
#if defined(USE_BLACKBOX) || defined(USE_BACKCHANNEL) || defined(USE_DASHBOARD)
            // signalling/sending to the message queue is not free, so, in this time-critical function, we only do it if necessary
            ++_send_blackbox_message_count;
            if (_send_blackbox_message_count >= _send_blackbox_message_denominator) {
                send_blackbox_message_count = 0;
                const ahrs_data_t ahrs_data = ahrs_data_t { 
                    .acc_gyro_rps =  acc_gyro_rps,
                    .gyro_rps_unfiltered = gyro_rps_unfiltered,
                    .orientation = orientation,
                    .delta_t = delta_t,
                    .time_microseconds = _time_microseconds,
                    .filler = {}
                };
                // typical SIGNAL or SEND overhead is ~50 CPU cycles for 60-byte data item
                if (_context.vehicle_controller.is_blackbox_active()) {
                    _context.ahrs_message_queue.SIGNAL(ahrs_data);
                }
#if defined(USE_BACKCHANNEL) || defined(USE_DASHBOARD)
                _context.ahrs_message_queue.SEND_AHRS_DATA(ahrs_data);
#endif
            }
#endif
        motor_commands_t motor_commands = _context.vehicle_controller.calculate_motor_commands(acc_gyro_rps.gyro_rps, orientation, delta_t, _context.debug);
        _context.motor_mixer_message_queue.SIGNAL(motor_commands);
    }
}

/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
[[noreturn]] void AhrsTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    if (_task_interval_microseconds == 0) {
        // interrupt driven scheduling
        while (true) {
            // wait until the IMU signals it has read some data
            _context.ahrs.get_imu_mutable().WAIT_IMU_DATA_READY();
            loop();
        }
    } else {
        // timer driven scheduling
        const uint32_t task_interval_ticks = _task_interval_microseconds < 1000 ? 1 : pdMS_TO_TICKS(_task_interval_microseconds / 1000);
        _previous_wake_time_ticks = xTaskGetTickCount();
        while (true) {
            // delay until the end of the next task_interval_ticks
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const BaseType_t was_delayed = xTaskDelayUntil(&_previous_wake_time_ticks, task_interval_ticks);
            if (was_delayed) {
                _was_delayed = true;
            }
#else
            vTaskDelayUntil(&_previous_wake_time_ticks, task_interval_ticks);
#endif
            loop();
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for Ahrs::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void AhrsTask::task_static(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<AhrsTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
