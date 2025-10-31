#include "BlackboxMessageQueue.h"
#include "FC_TelemetryData.h"
#include "FlightController.h"

#include <AHRS.h>
#include <Debug.h>
#include <Defaults.h>
#include <IMU_FiltersBase.h>
#include <IMU_Null.h>
#include <NonVolatileStorage.h>
#include <ReceiverNull.h>
#include <SensorFusion.h>

#include <string>

#include <unity.h>

#if !defined(OUTPUT_TO_MOTORS_DENOMINATOR)
//enum { OUTPUT_TO_MOTORS_DENOMINATOR = 1 };
static const uint32_t OUTPUT_TO_MOTORS_DENOMINATOR = 1;
#endif

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(hicpp-signed-bitwise,misc-const-correctness)
void test_flight_controller()
{
    static MadgwickFilter sensorFusionFilter;
    static IMU_Null imu(IMU_Base::XPOS_YPOS_ZPOS);
    static IMU_FiltersBase imuFilters;

    enum { MOTOR_COUNT = 4 };
    static Debug debug;
    static MotorMixerBase motorMixer(MOTOR_COUNT, debug);
    static ReceiverNull receiver;
    BlackboxMessageQueue blackboxMessageQueue;
    FlightController fc(AHRS_TASK_INTERVAL_MICROSECONDS, OUTPUT_TO_MOTORS_DENOMINATOR, motorMixer, blackboxMessageQueue, debug);
    static AHRS ahrs(AHRS::TIMER_DRIVEN, fc, sensorFusionFilter, imu, imuFilters);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
    TEST_ASSERT_FALSE(fc.motorsIsOn());

    fc.motorsSwitchOn();
    TEST_ASSERT_FALSE(fc.motorsIsOn());
    ahrs.setSensorFusionInitializing(false);
    fc.motorsSwitchOn();
    TEST_ASSERT_TRUE(fc.motorsIsOn());

    fc.motorsToggleOnOff();
    TEST_ASSERT_FALSE(fc.motorsIsOn());
    fc.motorsToggleOnOff();
    TEST_ASSERT_TRUE(fc.motorsIsOn());

    fc.motorsSwitchOff();
    TEST_ASSERT_FALSE(fc.motorsIsOn());
    fc.motorsSwitchOn();
    TEST_ASSERT_TRUE(fc.motorsIsOn());

    static const std::string pidNameSpeed = fc.getPID_Name(FlightController::ROLL_RATE_DPS);
    TEST_ASSERT_TRUE(pidNameSpeed.compare("ROLL_RATE") == 0);

    static const std::string pidNamePosition = fc.getPID_Name(FlightController::PITCH_RATE_DPS);
    TEST_ASSERT_TRUE(pidNamePosition.compare("PITCH_RATE") == 0);

    static const std::string pidNameYaw = fc.getPID_Name(FlightController::YAW_RATE_DPS);
    TEST_ASSERT_TRUE(pidNameYaw.compare("YAW_RATE") == 0);

    static const std::string pidNameRoll = fc.getPID_Name(FlightController::ROLL_ANGLE_DEGREES);
    TEST_ASSERT_TRUE(pidNameRoll.compare("ROLL_ANGLE") == 0);

    static const std::string pidNamePitch = fc.getPID_Name(FlightController::PITCH_ANGLE_DEGREES);
    TEST_ASSERT_TRUE(pidNamePitch.compare("PITCH_ANGLE") == 0);

    FlightController::tpa_config_t tpaConfig = fc.getTPA_Config();
    TEST_ASSERT_EQUAL(0, tpaConfig.tpa_breakpoint);
    TEST_ASSERT_EQUAL(0, tpaConfig.tpa_mode);
    TEST_ASSERT_EQUAL(0, tpaConfig.tpa_rate);

    static NonVolatileStorage nvs;
    nvs.init();

    tpaConfig = nvs.loadFlightControllerTPA_Config(NonVolatileStorage::DEFAULT_PID_PROFILE);
    fc.setTPA_Config(tpaConfig);

    tpaConfig = fc.getTPA_Config();
    TEST_ASSERT_EQUAL(DEFAULTS::flightControllerTPA_Config.tpa_breakpoint, tpaConfig.tpa_breakpoint);
    TEST_ASSERT_EQUAL(DEFAULTS::flightControllerTPA_Config.tpa_mode, tpaConfig.tpa_mode);
    TEST_ASSERT_EQUAL(DEFAULTS::flightControllerTPA_Config.tpa_rate, tpaConfig.tpa_rate);
}

void test_flight_controller_pid_indexes()
{
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::ROLL_RATE_DPS) == static_cast<int>(TD_FC_PIDS::ROLL_RATE_DPS));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::PITCH_RATE_DPS) == static_cast<int>(TD_FC_PIDS::PITCH_RATE_DPS));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::YAW_RATE_DPS) == static_cast<int>(TD_FC_PIDS::YAW_RATE_DPS));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::ROLL_ANGLE_DEGREES) == static_cast<int>(TD_FC_PIDS::ROLL_ANGLE_DEGREES));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::PITCH_ANGLE_DEGREES) == static_cast<int>(TD_FC_PIDS::PITCH_ANGLE_DEGREES));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::ROLL_SIN_ANGLE) == static_cast<int>(TD_FC_PIDS::ROLL_SIN_ANGLE));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::PITCH_SIN_ANGLE) == static_cast<int>(TD_FC_PIDS::PITCH_SIN_ANGLE));
}

void test_flight_controller_flight_mode_flags()
{
    TEST_ASSERT_EQUAL(FlightController::ANGLE_MODE, 1U << FlightController::LOG2_ANGLE_MODE);
    TEST_ASSERT_EQUAL(FlightController::HORIZON_MODE, 1U << FlightController::LOG2_HORIZON_MODE);
    TEST_ASSERT_EQUAL(FlightController::MAG_MODE, 1U << FlightController::LOG2_MAG_MODE);
    TEST_ASSERT_EQUAL(FlightController::ALT_HOLD_MODE, 1U << FlightController::LOG2_ALT_HOLD_MODE);
    TEST_ASSERT_EQUAL(FlightController::GPS_HOME_MODE, 1U << FlightController::LOG2_GPS_HOME_MODE);
    TEST_ASSERT_EQUAL(FlightController::GPS_HOLD_MODE, 1U << FlightController::LOG2_GPS_HOLD_MODE);
    TEST_ASSERT_EQUAL(FlightController::HEADFREE_MODE, 1U << FlightController::LOG2_HEADFREE_MODE);
    TEST_ASSERT_EQUAL(FlightController::PASSTHRU_MODE, 1U << FlightController::LOG2_PASSTHRU_MODE);
    TEST_ASSERT_EQUAL(FlightController::RANGEFINDER_MODE, 1U << FlightController::LOG2_RANGEFINDER_MODE);
    TEST_ASSERT_EQUAL(FlightController::FAILSAFE_MODE, 1U << FlightController::LOG2_FAILSAFE_MODE);
    TEST_ASSERT_EQUAL(FlightController::GPS_RESCUE_MODE, 1U << FlightController::LOG2_GPS_RESCUE_MODE);
}
// NOLINTEND(hicpp-signed-bitwise,misc-const-correctness)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_flight_controller);
    RUN_TEST(test_flight_controller_pid_indexes);
    RUN_TEST(test_flight_controller_flight_mode_flags);

    UNITY_END();
}
