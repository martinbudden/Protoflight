#include "Autopilot.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "MSP_Protoflight.h"
#include "NonVolatileStorage.h"

#include <FC_Telemetry.h>
#include <FC_TelemetryData.h>
#include <MSP_Protocol.h>
#include <MSP_Serial.h>
#include <MSP_Stream.h>
#include <RC_Modes.h>

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
    .rateLimits = { rates_t::LIMIT_MAX, rates_t::LIMIT_MAX, rates_t::LIMIT_MAX},
    .rcRates = { 7, 7, 7 },
    .rcExpos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttleMidpoint = 50,
    .throttleExpo = 0,
    .throttleLimitType = rates_t::THROTTLE_LIMIT_TYPE_OFF,
    .throttleLimitPercent = 100,
    //.ratesType = rates_t::RATES_TYPE_ACTUAL
};

static NonVolatileStorage nvs;
static MadgwickFilter sensorFusionFilter;
static ImuNull imu;
static Debug debug;
static constexpr uint8_t OUTPUT_TO_MOTORS_DENOMINATOR = 1;
static constexpr size_t MOTOR_COUNT = 4;
static constexpr size_t SERVO_COUNT = 0;
static IMU_Filters imuFilters(0.0F);
static MotorMixerBase motorMixer(MotorMixerBase::QUAD_X, OUTPUT_TO_MOTORS_DENOMINATOR, MOTOR_COUNT, SERVO_COUNT);
static ReceiverVirtual receiver;
static RcModes rc_modes;
static AhrsMessageQueue ahrsMessageQueue;
static FlightController fc(AHRS_TASK_INTERVAL_MICROSECONDS);
static Ahrs ahrs(Ahrs::TIMER_DRIVEN, sensorFusionFilter, imu);
static Autopilot autopilot(ahrsMessageQueue);
static Cockpit cockpit(autopilot, nullptr);
static msp_parameter_group_t pg = {
    .ahrs = ahrs,
    .flightController = fc,
    .ahrsMessageQueue = ahrsMessageQueue,
    .motorMixer = motorMixer,
    .cockpit = cockpit,
    .receiver = receiver,
    .rc_modes = rc_modes,
    .imuFilters = imuFilters,
    .debug = debug,
    .nonVolatileStorage = nvs,
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
    pack_telemetry_data_msp(&buf[0], ID, 0, pg, msp, MSP_API_VERSION); // 0, 1, 47
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
