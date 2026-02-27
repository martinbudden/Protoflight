#include "Cockpit.h"
#include "FC_TelemetryData.h"
#include "FlightController.h"
#include "NonVolatileStorage.h"

#include <ahrs.h>
#include <debug.h>
#include <imu_filters_base.h>
#include <imu_null.h>
#include <motor_mixer_base.h>
#include <receiver_virtual.h>
#include <sensor_fusion.h>

#include <string>

#include <unity.h>


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
    static ImuNull imu(ImuBase::XPOS_YPOS_ZPOS);

    static constexpr uint8_t OUTPUT_TO_MOTORS_DENOMINATOR = 1;
    static constexpr size_t MOTOR_COUNT = 4;
    static constexpr size_t SERVO_COUNT = 0;
    static MotorMixerBase motorMixer(MotorMixerBase::QUAD_X, OUTPUT_TO_MOTORS_DENOMINATOR, MOTOR_COUNT, SERVO_COUNT);
    static ReceiverVirtual receiver;
    FlightController fc(AHRS_TASK_INTERVAL_MICROSECONDS);
    static Ahrs ahrs(Ahrs::TIMER_DRIVEN, sensorFusionFilter, imu);
    //TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
    TEST_ASSERT_FALSE(motorMixer.motors_is_on());

    fc.motorsSwitchOn(motorMixer); // should not switch on, since sensor fusion filter is initializing
    TEST_ASSERT_FALSE(motorMixer.motors_is_on());
    fc.set_sensor_fusion_filter_is_initializing(false);
    fc.motorsSwitchOn(motorMixer);
    TEST_ASSERT_TRUE(motorMixer.motors_is_on());

    fc.motorsSwitchOff(motorMixer);
    TEST_ASSERT_FALSE(motorMixer.motors_is_on());
    fc.motorsSwitchOn(motorMixer);
    TEST_ASSERT_TRUE(motorMixer.motors_is_on());

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

    tpa_config_t tpaConfig = fc.getTPA_Config();
    TEST_ASSERT_EQUAL(0, tpaConfig.tpa_breakpoint);
    TEST_ASSERT_EQUAL(0, tpaConfig.tpa_mode);
    TEST_ASSERT_EQUAL(0, tpaConfig.tpa_rate);

    static NonVolatileStorage nvs;
    nvs.init();

    tpaConfig = nvs.load_flight_controller_tpa_config(NonVolatileStorage::DEFAULT_PID_PROFILE);
    fc.setTPA_Config(tpaConfig);

    tpaConfig = fc.getTPA_Config();

    TEST_ASSERT_EQUAL(1350, tpaConfig.tpa_breakpoint);
    TEST_ASSERT_EQUAL(FlightController::TPA_MODE_D, tpaConfig.tpa_mode);
    TEST_ASSERT_EQUAL(65, tpaConfig.tpa_rate);
}

void test_flight_controller_pid_indexes()
{
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::ROLL_RATE_DPS) == static_cast<int>(TD_FC_PIDS::ROLL_RATE_DPS));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::PITCH_RATE_DPS) == static_cast<int>(TD_FC_PIDS::PITCH_RATE_DPS));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::YAW_RATE_DPS) == static_cast<int>(TD_FC_PIDS::YAW_RATE_DPS));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::ROLL_ANGLE_DEGREES) == static_cast<int>(TD_FC_PIDS::ROLL_ANGLE_DEGREES));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::PITCH_ANGLE_DEGREES) == static_cast<int>(TD_FC_PIDS::PITCH_ANGLE_DEGREES));
#if defined(USE_SIN_ANGLE_PIDS)
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::ROLL_SIN_ANGLE) == static_cast<int>(TD_FC_PIDS::ROLL_SIN_ANGLE));
    TEST_ASSERT_TRUE(static_cast<int>(FlightController::PITCH_SIN_ANGLE) == static_cast<int>(TD_FC_PIDS::PITCH_SIN_ANGLE));
#endif
}

void test_flight_controller_flight_mode_flags()
{
    TEST_ASSERT_EQUAL(Cockpit::ANGLE_MODE, 1U << Cockpit::LOG2_ANGLE_MODE);
    TEST_ASSERT_EQUAL(Cockpit::HORIZON_MODE, 1U << Cockpit::LOG2_HORIZON_MODE);
    TEST_ASSERT_EQUAL(Cockpit::MAG_MODE, 1U << Cockpit::LOG2_MAG_MODE);
    TEST_ASSERT_EQUAL(Cockpit::ALTITUDE_HOLD_MODE, 1U << Cockpit::LOG2_ALTITUDE_HOLD_MODE);
    // TEST_ASSERT_EQUAL(Cockpit::GPS_HOME_MODE, 1U << Cockpit::LOG2_GPS_HOME_MODE);
    TEST_ASSERT_EQUAL(Cockpit::POSITION_HOLD_MODE, 1U << Cockpit::LOG2_POSITION_HOLD_MODE);
    TEST_ASSERT_EQUAL(Cockpit::HEADFREE_MODE, 1U << Cockpit::LOG2_HEADFREE_MODE);
    TEST_ASSERT_EQUAL(Cockpit::CHIRP_MODE, 1U << Cockpit::LOG2_CHIRP_MODE);
    TEST_ASSERT_EQUAL(Cockpit::PASSTHRU_MODE, 1U << Cockpit::LOG2_PASSTHRU_MODE);
    // TEST_ASSERT_EQUAL(Cockpit::RANGEFINDER_MODE, 1U << Cockpit::LOG2_RANGEFINDER_MODE);
    TEST_ASSERT_EQUAL(Cockpit::FAILSAFE_MODE, 1U << Cockpit::LOG2_FAILSAFE_MODE);
    TEST_ASSERT_EQUAL(Cockpit::GPS_RESCUE_MODE, 1U << Cockpit::LOG2_GPS_RESCUE_MODE);
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
