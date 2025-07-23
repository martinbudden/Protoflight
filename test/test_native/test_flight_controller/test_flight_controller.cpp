#include "FC_TelemetryData.h"
#include "FlightController.h"

#include <AHRS.h>
#include <IMU_FiltersBase.h>
#include <IMU_Null.h>
#include <MotorMixerBase.h>
#include <RadioController.h>
#include <ReceiverNull.h>
#include <SensorFusion.h>

#include <string>

#include <unity.h>

#if !defined(FC_TASK_INTERVAL_MICROSECONDS)
enum { FC_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif

class IMU_FiltersNull : public IMU_FiltersBase
{
public:
    virtual ~IMU_FiltersNull() = default;
    IMU_FiltersNull() = default;
    void setFilters() override {};
    void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override { (void)gyroRPS; (void)acc; (void)deltaT; }
    // IMU_FiltersNull is not copyable or moveable
    IMU_FiltersNull(const IMU_FiltersNull&) = delete;
    IMU_FiltersNull& operator=(const IMU_FiltersNull&) = delete;
    IMU_FiltersNull(IMU_FiltersNull&&) = delete;
    IMU_FiltersNull& operator=(IMU_FiltersNull&&) = delete;
};

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(misc-const-correctness)
void test_flight_controller()
{
    static MadgwickFilter sensorFusionFilter;
    static IMU_Null imu(IMU_Base::XPOS_YPOS_ZPOS);
    static IMU_FiltersNull imuFilters;
    static AHRS ahrs(AHRS_TASK_INTERVAL_MICROSECONDS, sensorFusionFilter, imu, imuFilters);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());

    enum { MOTOR_COUNT = 4 };
    static MotorMixerBase motorMixer(MOTOR_COUNT);
    static ReceiverNull receiver;
    static RadioController radioController(receiver);
    FlightController fc(FC_TASK_INTERVAL_MICROSECONDS, motorMixer, ahrs, radioController);
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
    // NOLINTBEGIN(hicpp-signed-bitwise)
    TEST_ASSERT_EQUAL(FlightController::ANGLE_MODE, 1U << FlightController::LOG2_ANGLE_MODE);
    TEST_ASSERT_EQUAL(FlightController::HORIZON_MODE, 1U << FlightController::LOG2_ANGLE_MODE);
    TEST_ASSERT_EQUAL(FlightController::MAG_MODE, 1U << FlightController::LOG2_ANGLE_MODE);
    TEST_ASSERT_EQUAL(FlightController::ALT_HOLD_MODE, 1U << FlightController::LOG2_ALT_HOLD_MODE);
    TEST_ASSERT_EQUAL(FlightController::GPS_HOME_MODE, 1U << FlightController::LOG2_GPS_HOME_MODE);
    TEST_ASSERT_EQUAL(FlightController::GPS_HOLD_MODE, 1U << FlightController::LOG2_GPS_HOLD_MODE);
    TEST_ASSERT_EQUAL(FlightController::HEADFREE_MODE, 1U << FlightController::LOG2_HEADFREE_MODE);
    TEST_ASSERT_EQUAL(FlightController::PASSTHRU_MODE, 1U << FlightController::LOG2_PASSTHRU_MODE);
    TEST_ASSERT_EQUAL(FlightController::RANGEFINDER_MODE, 1U << FlightController::LOG2_RANGEFINDER_MODE);
    TEST_ASSERT_EQUAL(FlightController::FAILSAFE_MODE, 1U << FlightController::LOG2_FAILSAFE_MODE);
    TEST_ASSERT_EQUAL(FlightController::GPS_RESCUE_MODE, 1U << FlightController::LOG2_GPS_RESCUE_MODE);
    // NOLINTEND(hicpp-signed-bitwise)
}
// NOLINTEND(misc-const-correctness)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_flight_controller);
    RUN_TEST(test_flight_controller_pid_indexes);

    UNITY_END();
}
