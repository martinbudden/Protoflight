#include "autopilot.h"
#include "fc_telemetry_data.h"
#include "flight_controller.h"
#include "imu_filters.h"
#include "msp_protoflight.h"
#include "non_volatile_storage.h"

#include <cockpit.h>
#include <rc_modes.h>

#include <ahrs.h>
#include <ahrs_message_queue.h>
#include <debug.h>
#include <imu_null.h>
#include <motor_mixer_base.h>
#include <motor_mixer_message_queue.h>
#include <receiver_virtual.h>
#include <sensor_fusion.h>

#include <unity.h>

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif
#if !defined(OUTPUT_TO_MOTORS_DENOMINATOR)
enum { OUTPUT_TO_MOTORS_DENOMINATOR = 2 }; // runs at half rate of AHRS_TASK
#endif


void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)
static NonVolatileStorage nvs;
static MadgwickFilter sensorFusionFilter;
static ImuNull imu;
static Debug debug;
static constexpr size_t MOTOR_COUNT = 4;
static constexpr size_t SERVO_COUNT = 0;
static ImuFilters imu_filters(0.0F);
static MotorMixerBase motor_mixer(MotorMixerBase::QUAD_X, OUTPUT_TO_MOTORS_DENOMINATOR, MOTOR_COUNT, SERVO_COUNT);
static ReceiverVirtual receiver;
static RcModes rc_modes;
static AhrsMessageQueue ahrs_message_queue;
static FlightController flight_controller(AHRS_TASK_INTERVAL_MICROSECONDS);
static Ahrs ahrs(Ahrs::TIMER_DRIVEN, sensorFusionFilter, imu);
static Autopilot autopilot(ahrs_message_queue);
static Cockpit cockpit(autopilot, nullptr);
static const float looptime_seconds = 0.001F;
static RpmFilters rpm_filters(MOTOR_COUNT, looptime_seconds);
static MotorMixerMessageQueue motor_mixer_message_queue {};
static motor_commands_t motor_commands {};


void test_main_control_loop()
{
    const float delta_t = looptime_seconds;
    const uint32_t time_microseconds = 1000;
    const uint32_t time_microseconds_delta = 1000;
    const uint32_t tick_count = 1;
    const uint32_t tick_count_delta = 1;

    // Receiver task loop, update rc_modes and flight_controller setpoints with data from receiver
    receiver.update(tick_count_delta);
    static receiver_context_t receiver_context = {
        .rc_modes = rc_modes,
        .flight_controller = flight_controller,
        .motor_mixer = motor_mixer,
        .debug = debug,
        .blackbox = nullptr,
        .osd = nullptr
    };
    cockpit.update_controls(tick_count, receiver, receiver_context); // this errors
    cockpit.check_failsafe(tick_count, receiver_context);

    const ahrs_data_t& ahrs_data = ahrs.read_imu_and_update_orientation(time_microseconds, time_microseconds_delta, imu_filters, flight_controller, debug);
    flight_controller.update_outputs_using_pids(ahrs_data, ahrs_message_queue, motor_mixer_message_queue, debug);

    motor_mixer_message_queue.WAIT(motor_commands);
    motor_mixer.output_to_motors(motor_commands, &rpm_filters, delta_t, tick_count, debug);

    TEST_ASSERT_TRUE(true);
}

// NOLINTEND(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_main_control_loop);

    UNITY_END();
}
