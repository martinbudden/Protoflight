#include "Cockpit.h"
#include "FC_TelemetryData.h"
#include "IMU_Filters.h"
#include "MSP_Protoflight.h"
#include "NonVolatileStorage.h"

#include <AHRS.h>
#include <AHRS_MessageQueue.h>
#include <Debug.h>
#include <FC_Telemetry.h>
#include <IMU_Null.h>
#include <MSP_Protocol.h>
#include <MotorMixerBase.h>
#include <ReceiverVirtual.h>
#include <SV_TelemetryData.h>
#include <SensorFusion.h>

#include <string>

#include <unity.h>

#if !defined(OUTPUT_TO_MOTORS_DENOMINATOR)
enum { OUTPUT_TO_MOTORS_DENOMINATOR = 2 };
#endif

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

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(misc-const-correctness)
void test_telemetry_msp()
{
    static MadgwickFilter sensorFusionFilter;
    static IMU_Null imu(IMU_Base::XPOS_YPOS_ZPOS);
    static NonVolatileStorage nvs;

    enum { MOTOR_COUNT = 4, SERVO_COUNT = 0 };
    static Debug debug;
    static IMU_Filters imuFilters(MOTOR_COUNT, debug, 0.0F);
    static MotorMixerBase motorMixer(MotorMixerBase::QUAD_X, MOTOR_COUNT, SERVO_COUNT, &debug);
    static ReceiverVirtual receiver;
    static RcModes rc_modes;
    static AHRS_MessageQueue ahrsMessageQueue;
    static FlightController flightController(AHRS_TASK_INTERVAL_MICROSECONDS, OUTPUT_TO_MOTORS_DENOMINATOR, motorMixer, ahrsMessageQueue, debug);
    static AHRS ahrs(AHRS::TIMER_DRIVEN, flightController, sensorFusionFilter, imu, imuFilters);
    static Autopilot autopilot(ahrsMessageQueue);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
    static Cockpit cockpit(rc_modes, flightController, autopilot, imuFilters, debug, nullptr);

    // statically allocate an MSP object
    static MSP_Protoflight msp(ahrs, flightController, cockpit, receiver, rc_modes, imuFilters, debug, nvs, nullptr, nullptr, nullptr, nullptr);
//size_t packTelemetryData_MSP(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, MSP_Base& msp, int16_t cmdMSP)
    static std::array<uint8_t, 256> buf;
    enum { ID = 0x11223344 };
    packTelemetryData_MSP(&buf[0], ID, 0, msp, MSP_API_VERSION); // 0, 1, 47
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
// NOLINTEND(misc-const-correctness)


int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_telemetry_msp);

    UNITY_END();
}
