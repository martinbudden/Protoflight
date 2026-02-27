#include "Autopilot.h"
#include "FC_TelemetryData.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "MSP_Protoflight.h"
#include "NonVolatileStorage.h"

#include <Cockpit.h>
#include <RC_Modes.h>

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


void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)
static NonVolatileStorage nvs;
static MadgwickFilter sensorFusionFilter;
static ImuNull imu;
static Debug debug;
static constexpr uint8_t OUTPUT_TO_MOTORS_DENOMINATOR = 1;
static constexpr size_t MOTOR_COUNT = 4;
static constexpr size_t SERVO_COUNT = 0;
static IMU_Filters imuFilters(0.0F);
static MotorMixerBase motor_mixer(MotorMixerBase::QUAD_X, OUTPUT_TO_MOTORS_DENOMINATOR, MOTOR_COUNT, SERVO_COUNT);
static ReceiverVirtual receiver;
static RcModes rc_modes;
static AhrsMessageQueue ahrsMessageQueue;
static FlightController flightController(AHRS_TASK_INTERVAL_MICROSECONDS);
static Ahrs ahrs(Ahrs::TIMER_DRIVEN, sensorFusionFilter, imu);
static Autopilot autopilot(ahrsMessageQueue);
static Cockpit cockpit(autopilot, nullptr);
static const float looptime_seconds = 0.001F;
static RpmFilters rpm_filters(MOTOR_COUNT, looptime_seconds);;
static MotorMixerMessageQueue motor_mixer_message_queue {};
static motor_mixer_message_queue_item_t motor_mixer_message_queue_item {};


void test_main_control_loop()
{
    const float delta_t = looptime_seconds;
    const uint32_t time_microseconds = 1000;
    const uint32_t time_microseconds_delta = 1000;
    const uint32_t tick_count = 1;
    const uint32_t tick_count_delta = 1;

    // Receiver task loop, update rc_modes and flightController setpoints with data from receiver
    receiver.update(tick_count_delta);
    static receiver_parameter_group_t receiver_parameter_group = {
        .rc_modes = rc_modes,
        .flight_controller = flightController,
        .motor_mixer = motor_mixer,
        .debug = debug,
        .blackbox = nullptr,
        .osd = nullptr
    };
    cockpit.update_controls(tick_count, receiver, receiver_parameter_group); // this errors
    cockpit.check_failsafe(tick_count, receiver_parameter_group);

    const ahrs_data_t& ahrsData = ahrs.read_imu_and_update_orientation(time_microseconds, time_microseconds_delta, imuFilters, flightController, debug);
    flightController.update_outputs_using_pids(ahrsData, ahrsMessageQueue, motor_mixer_message_queue, debug);

    motor_mixer_message_queue.WAIT(motor_mixer_message_queue_item);
    motor_mixer.output_to_motors(motor_mixer_message_queue_item, &rpm_filters, delta_t, tick_count, debug);

    TEST_ASSERT_TRUE(true);
}

// NOLINTEND(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_main_control_loop);

    UNITY_END();
}
