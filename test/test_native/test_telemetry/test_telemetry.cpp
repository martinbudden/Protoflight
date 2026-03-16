#include "autopilot.h"
#include "cockpit.h"
#include "flight_controller.h"
#include "imu_filters.h"
#include "msp_protoflight.h"
#include "non_volatile_storage.h"

#include <fc_telemetry.h>
#include <fc_telemetry_data.h>
#include <msp_protocol.h>
#include <msp_serial.h>
#include <msp_stream.h>
#include <rc_modes.h>

#include <ahrs.h>
#include <ahrs_message_queue.h>
#include <debug.h>
#include <imu_null.h>
#include <motor_mixer_base.h>
#include <receiver_virtual.h>
#include <sensor_fusion.h>

#include <string>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif

static const rates_t cockpitRates {
    .rate_limits = { rates_t::LIMIT_MAX, rates_t::LIMIT_MAX, rates_t::LIMIT_MAX},
    .rc_rates = { 7, 7, 7 },
    .rc_expos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttle_midpoint = 50,
    .throttle_expo = 0,
    .throttle_limit_type = rates_t::THROTTLE_LIMIT_TYPE_OFF,
    .throttle_limit_percent = 100,
    //.ratesType = rates_t::RATES_TYPE_ACTUAL
};

static NonVolatileStorage nvs;
static MadgwickFilter sensorFusionFilter;
static ImuNull imu;
static Debug debug;
static constexpr uint8_t OUTPUT_TO_MOTORS_DENOMINATOR = 1;
static constexpr size_t MOTOR_COUNT = 4;
static constexpr size_t SERVO_COUNT = 0;
static ImuFilters imu_filters(0.0F);
static MotorMixerBase motor_mixer(MotorMixerBase::QUAD_X, OUTPUT_TO_MOTORS_DENOMINATOR, MOTOR_COUNT, SERVO_COUNT);
static ReceiverVirtual receiver;
static RcModes rc_modes;
static AhrsMessageQueue ahrs_message_queue;
static FlightController fc(AHRS_TASK_INTERVAL_MICROSECONDS);
static Ahrs ahrs(Ahrs::TIMER_DRIVEN, sensorFusionFilter, imu);
static Autopilot autopilot(ahrs_message_queue);
static Cockpit cockpit(autopilot, nullptr);
static msp_context_t ctx = {
    .ahrs = ahrs,
    .flight_controller = fc,
    .ahrs_message_queue = ahrs_message_queue,
    .motor_mixer = motor_mixer,
    .cockpit = cockpit,
    .receiver = receiver,
    .rc_modes = rc_modes,
    .imu_filters = imu_filters,
    .debug = debug,
    .nvs = nvs,
    .blackbox = nullptr,
    .vtx = nullptr,
    .osd = nullptr,
    .gps = nullptr
};

void test_telemetry_msp()
{
    // statically allocate an MSP object
    static MSP_Protoflight msp;
    static std::array<uint8_t, 256> buf;
    enum { ID = 0x11223344 };
    pack_telemetry_data_msp(&buf[0], ID, 0, ctx, msp, MSP_API_VERSION); // 0, 1, 47
    TEST_ASSERT_EQUAL(0x44, buf[0]);
    TEST_ASSERT_EQUAL(0x33, buf[1]);
    TEST_ASSERT_EQUAL(0x22, buf[2]);
    TEST_ASSERT_EQUAL(0x11, buf[3]);

    TEST_ASSERT_EQUAL('$', buf[4]);
    TEST_ASSERT_EQUAL('M', buf[5]);
    TEST_ASSERT_EQUAL('>', buf[6]);
    TEST_ASSERT_EQUAL(3, buf[7]); // payload length
    TEST_ASSERT_EQUAL(MSP_API_VERSION, buf[8]); // messageType
    TEST_ASSERT_EQUAL(0, buf[9]); // payload
    TEST_ASSERT_EQUAL(1, buf[10]);
    TEST_ASSERT_EQUAL(47, buf[11]);
    TEST_ASSERT_EQUAL(46, buf[12]); // checksum
}

void test_flight_controller_pid_indexes()
{
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::ROLL_RATE_DPS) == static_cast<int>(TD_FC_PIDS::ROLL_RATE_DPS));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::PITCH_RATE_DPS) == static_cast<int>(TD_FC_PIDS::PITCH_RATE_DPS));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::YAW_RATE_DPS) == static_cast<int>(TD_FC_PIDS::YAW_RATE_DPS));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::ROLL_ANGLE_DEGREES) == static_cast<int>(TD_FC_PIDS::ROLL_ANGLE_DEGREES));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::PITCH_ANGLE_DEGREES) == static_cast<int>(TD_FC_PIDS::PITCH_ANGLE_DEGREES));
}
// NOLINTEND(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)


int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_telemetry_msp);

    UNITY_END();
}
