#include "Autopilot.h"
#include "Cockpit.h"
#include "FC_TelemetryData.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "MSP_Protoflight.h"
#include "NonVolatileStorage.h"

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
static MSP_Protoflight msp;
static MspStream mspStream(msp);


void test_msp_set_failsafe_config()
{
    mspStream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payload_size = 8;
    const uint8_t checksum = 241;
    const failsafe_config_t fsIn {
        .throttle_pwm = 1963,
        .throttle_low_delay_deciseconds = 4107,
        .recovery_delay_deciseconds = 5, // cppcheck-suppress unusedStructMember
        .delay_deciseconds = 3,
        .landing_time_seconds = 5,
        .procedure = 13,
        .switch_mode = 9,
        .stick_threshold_percent = 30, // cppcheck-suppress unusedStructMember
    };
    const std::array<uint8_t, 13> inStream = {
        'M', '<', payload_size, MSP_SET_FAILSAFE_CONFIG,
        fsIn.delay_deciseconds,
        fsIn.landing_time_seconds,
        static_cast<uint8_t>(fsIn.throttle_pwm & 0xFFU),
        static_cast<uint8_t>(fsIn.throttle_pwm >> 8U),
        fsIn.switch_mode,
        static_cast<uint8_t>(fsIn.throttle_low_delay_deciseconds & 0xFFU),
        static_cast<uint8_t>(fsIn.throttle_low_delay_deciseconds >> 8U),
        fsIn.procedure,
        checksum
    };

    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.put_char(pg, inChar, &pwh);
        if (eof) {
            break;
        }
    }
    TEST_ASSERT_EQUAL(checksum, mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_IDLE, mspStream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(76, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.data_len);

    const failsafe_config_t fsOut = cockpit.getFailsafeConfig();
    TEST_ASSERT_EQUAL(fsIn.throttle_pwm, fsOut.throttle_pwm);
    TEST_ASSERT_EQUAL(fsIn.throttle_low_delay_deciseconds, fsOut.throttle_low_delay_deciseconds);
    TEST_ASSERT_EQUAL(fsIn.delay_deciseconds, fsOut.delay_deciseconds);
    TEST_ASSERT_EQUAL(fsIn.landing_time_seconds, fsOut.landing_time_seconds);
    TEST_ASSERT_EQUAL(fsIn.procedure, fsOut.procedure);
    TEST_ASSERT_EQUAL(fsIn.switch_mode, fsOut.switch_mode);
}

void test_msp_pid_in()
{
    std::array<uint8_t, 128> buf;
    StreamBufWriter sbuf(&buf[0], sizeof(buf));
    for (uint8_t ii = 0; ii < 3*FlightController::PID_COUNT; ++ii) {
        sbuf.write_u8(ii);
    }
    sbuf.reset();
    StreamBufReader sbufReader(sbuf);
    msp.process_set_command(pg, MSP_SET_PID, sbufReader);

    FlightController::PIDF_uint16_t pid16 = fc.get_pid_msp(FlightController::ROLL_RATE_DPS);
    TEST_ASSERT_EQUAL(0, pid16.kp);
    TEST_ASSERT_EQUAL(1, pid16.ki);
    TEST_ASSERT_EQUAL(2, pid16.kd);
    VehicleControllerBase::PIDF_uint16_t pid = fc.get_pid_constants(FlightController::ROLL_RATE_DPS);
    TEST_ASSERT_EQUAL(0, pid.kp);
    TEST_ASSERT_EQUAL(1, pid.ki);
    TEST_ASSERT_EQUAL(2, pid.kd);

    pid16 = fc.get_pid_msp(FlightController::PITCH_RATE_DPS);
    TEST_ASSERT_EQUAL(3, pid16.kp);
    TEST_ASSERT_EQUAL(4, pid16.ki);
    TEST_ASSERT_EQUAL(5, pid16.kd);
    pid = fc.get_pid_constants(FlightController::PITCH_RATE_DPS);
    TEST_ASSERT_EQUAL(3, pid.kp);
    TEST_ASSERT_EQUAL(4, pid.ki);
    TEST_ASSERT_EQUAL(5, pid.kd);
}

void test_msp_features()
{
    std::array<uint8_t, 128> buf;
    StreamBufWriter sbuf(&buf[0], sizeof(buf));
    msp.process_get_command(pg, MSP_FEATURE_CONFIG, sbuf);
    sbuf.reset();
    const uint32_t featuresRead = sbuf.read_u32();

    TEST_ASSERT_EQUAL(cockpit.enabledFeatures(), featuresRead);
}

void test_msp_raw_imu()
{
    imu.set_acc_raw({3, 5, 7});
    imu.set_gyro_raw({11, 13, 17});

    mspStream.set_packet_state(MSP_IDLE);

    mspStream.process_received_packet_data('M');
    TEST_ASSERT_EQUAL(MSP_HEADER_M, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());

    mspStream.process_received_packet_data('<'); // command packet
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, mspStream.get_packet_type());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());

    mspStream.process_received_packet_data(1); // size
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(1, mspStream.get_checksum1());

    mspStream.process_received_packet_data(MSP_RAW_IMU); // command
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(103, mspStream.get_checksum1());

    // checksum
    mspStream.process_received_packet_data(103);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());

    // any character after the checksum completes the command
    mspStream.process_received_packet_data(0);
    TEST_ASSERT_EQUAL(MSP_COMMAND_RECEIVED, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());

    msp_const_packet_t reply = mspStream.process_in_buf(pg);

    TEST_ASSERT_EQUAL(MSP_RAW_IMU, reply.cmd);
    const uint16_t accX = reply.payload.read_u16();
    const uint16_t accY = reply.payload.read_u16();
    const uint16_t accZ = reply.payload.read_u16();
    const uint16_t gyroX = reply.payload.read_u16();
    const uint16_t gyroY = reply.payload.read_u16();
    const uint16_t gyroZ = reply.payload.read_u16();
    const uint16_t magX = reply.payload.read_u16();
    const uint16_t magY = reply.payload.read_u16();
    const uint16_t magZ = reply.payload.read_u16();
    TEST_ASSERT_EQUAL(3, accX);
    TEST_ASSERT_EQUAL(5, accY);
    TEST_ASSERT_EQUAL(7, accZ);
    TEST_ASSERT_EQUAL(11, gyroX);
    TEST_ASSERT_EQUAL(13, gyroY);
    TEST_ASSERT_EQUAL(17, gyroZ);
    TEST_ASSERT_EQUAL(0, magX);
    TEST_ASSERT_EQUAL(0, magY);
    TEST_ASSERT_EQUAL(0, magZ);

    reply.payload.switch_to_reader(); // change StreamBufWriter direction
    const msp_stream_packet_with_header_t pwh = mspStream.serial_encode(reply, MSP_V1); // encode with MSP version 1

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(98, pwh.checksum);
    TEST_ASSERT_EQUAL(18, pwh.data_len);
}
// NOLINTEND(cert-err58-cpp,fuchsia-statically-constructed-objects,misc-const-correctness)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_msp_set_failsafe_config);

    RUN_TEST(test_msp_pid_in);
    RUN_TEST(test_msp_features);
    RUN_TEST(test_msp_raw_imu);

    UNITY_END();
}
