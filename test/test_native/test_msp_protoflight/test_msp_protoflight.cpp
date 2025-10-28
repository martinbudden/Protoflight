#include "BlackboxMessageQueue.h"
#include "FC_TelemetryData.h"
#include "Features.h"
#include "FlightController.h"
#include <AHRS.h>
#include <Debug.h>
#include <IMU_FiltersBase.h>
#include <IMU_Null.h>
#include <MSP_ProtoFlight.h>
#include <MSP_Protocol.h>
#include <MSP_Serial.h>
#include <MSP_Stream.h>
#include <MotorMixerBase.h>
#include <NonVolatileStorage.h>
#include <RadioController.h>
#include <ReceiverNull.h>
#include <SensorFusion.h>

#include <unity.h>

#if !defined(OUTPUT_TO_MOTORS_DENOMINATOR)
enum { OUTPUT_TO_MOTORS_DENOMINATOR = 1 };
#endif

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif


static const RadioController::rates_t radioControllerRates {
    .rateLimits = { RadioController::RATE_LIMIT_MAX, RadioController::RATE_LIMIT_MAX, RadioController::RATE_LIMIT_MAX},
    .rcRates = { 7, 7, 7 },
    .rcExpos = { 0, 0, 0 },
    .rates = { 67, 67, 67 },
    .throttleMidpoint = 50,
    .throttleExpo = 0,
    .throttleLimitType = RadioController::THROTTLE_LIMIT_TYPE_OFF,
    .throttleLimitPercent = 100,
    //.ratesType = RadioController::RATES_TYPE_ACTUAL
};

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(misc-const-correctness)
void test_msp_set_failsafe_config()
{
    static NonVolatileStorage nvs;
    static Features features;
    static MadgwickFilter sensorFusionFilter;
    static IMU_Null imu;
    static IMU_FiltersBase imuFilters;
    static AHRS ahrs(AHRS::TIMER_DRIVEN, sensorFusionFilter, imu, imuFilters);
    enum { MOTOR_COUNT = 4 };
    static Debug debug;
    static MotorMixerBase motorMixer(MOTOR_COUNT, debug);
    static ReceiverNull receiver;
    static BlackboxMessageQueue blackboxMessageQueue;
    static FlightController fc(AHRS_TASK_INTERVAL_MICROSECONDS, OUTPUT_TO_MOTORS_DENOMINATOR, ahrs, motorMixer, blackboxMessageQueue, debug);
    static RadioController radioController(receiver, fc, radioControllerRates);

    static MSP_ProtoFlight msp(ahrs, fc, radioController, debug, nvs, features);
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    const uint8_t payloadSize = 8;
    const uint8_t checksum = 197;
    const RadioController::failsafe_t fsIn {
        .delay = 3,
        .landing_time = 5,
        .switch_mode = 9,
        .procedure = 13,
        .throttle = 2963,
        .throttle_low_delay = 4107
    };
    const std::array<uint8_t, 13> inStream = {
        'M', '<', payloadSize, MSP_SET_FAILSAFE_CONFIG,
        fsIn.delay,
        fsIn.landing_time,
        static_cast<uint8_t>(fsIn.throttle & 0xFFU),
        static_cast<uint8_t>(fsIn.throttle >> 8U),
        fsIn.switch_mode,
        static_cast<uint8_t>(fsIn.throttle_low_delay & 0xFFU),
        static_cast<uint8_t>(fsIn.throttle_low_delay >> 8U),
        fsIn.procedure,
        checksum
    };

    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.putChar(inChar, &pwh);
        if (eof) {
            break;
        }
    }
    TEST_ASSERT_EQUAL(checksum, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(76, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.dataLen);

    const RadioController::failsafe_t fsOut = radioController.getFailsafe();
    TEST_ASSERT_EQUAL(fsIn.delay, fsOut.delay);
    TEST_ASSERT_EQUAL(fsIn.landing_time, fsOut.landing_time);
    TEST_ASSERT_EQUAL(fsIn.throttle, fsOut.throttle);
    TEST_ASSERT_EQUAL(fsIn.switch_mode, fsOut.switch_mode);
    TEST_ASSERT_EQUAL(fsIn.throttle_low_delay, fsOut.throttle_low_delay);
    TEST_ASSERT_EQUAL(fsIn.procedure, fsOut.procedure);
}

void test_msp_pid_in()
{
    static NonVolatileStorage nvs;
    static Features features;
    static MadgwickFilter sensorFusionFilter;
    static IMU_Null imu;
    static IMU_FiltersBase imuFilters;
    static AHRS ahrs(AHRS::TIMER_DRIVEN, sensorFusionFilter, imu, imuFilters);
    enum { MOTOR_COUNT = 4 };
    static Debug debug;
    static MotorMixerBase motorMixer(MOTOR_COUNT, debug);
    static ReceiverNull receiver;
    static BlackboxMessageQueue blackboxMessageQueue;
    static FlightController fc(AHRS_TASK_INTERVAL_MICROSECONDS, OUTPUT_TO_MOTORS_DENOMINATOR, ahrs, motorMixer, blackboxMessageQueue, debug);
    static RadioController radioController(receiver, fc, radioControllerRates);

    static MSP_ProtoFlight msp(ahrs, fc, radioController, debug, nvs, features);
    static const MSP_Stream mspStream(msp);

    std::array<uint8_t, 128> buf;
    StreamBuf sbuf(&buf[0], sizeof(buf));
    for (uint8_t ii = 0; ii < 3*FlightController::PID_COUNT; ++ii) {
        sbuf.writeU8(ii);
    }
    sbuf.reset();
    msp.processInCommand(MSP_SET_PID, sbuf);

    FlightController::PIDF_uint16_t pid16 = fc.getPID_MSP(FlightController::ROLL_RATE_DPS);
    TEST_ASSERT_EQUAL(0, pid16.kp);
    TEST_ASSERT_EQUAL(1, pid16.ki);
    TEST_ASSERT_EQUAL(2, pid16.kd);
    VehicleControllerBase::PIDF_uint16_t pid = fc.getPID_Constants(FlightController::ROLL_RATE_DPS);
    TEST_ASSERT_EQUAL(0, pid.kp);
    TEST_ASSERT_EQUAL(1, pid.ki);
    TEST_ASSERT_EQUAL(2, pid.kd);

    pid16 = fc.getPID_MSP(FlightController::PITCH_RATE_DPS);
    TEST_ASSERT_EQUAL(3, pid16.kp);
    TEST_ASSERT_EQUAL(4, pid16.ki);
    TEST_ASSERT_EQUAL(5, pid16.kd);
    pid = fc.getPID_Constants(FlightController::PITCH_RATE_DPS);
    TEST_ASSERT_EQUAL(3, pid.kp);
    TEST_ASSERT_EQUAL(4, pid.ki);
    TEST_ASSERT_EQUAL(5, pid.kd);
}

void test_msp_features()
{
    static NonVolatileStorage nvs;
    static Features features;
    static MadgwickFilter sensorFusionFilter;
    static IMU_Null imu;
    static IMU_FiltersBase imuFilters;
    static AHRS ahrs(AHRS::TIMER_DRIVEN, sensorFusionFilter, imu, imuFilters);
    enum { MOTOR_COUNT = 4 };
    static Debug debug;
    static MotorMixerBase motorMixer(MOTOR_COUNT, debug);
    static ReceiverNull receiver;
    static BlackboxMessageQueue blackboxMessageQueue;
    static FlightController fc(AHRS_TASK_INTERVAL_MICROSECONDS, OUTPUT_TO_MOTORS_DENOMINATOR, ahrs, motorMixer, blackboxMessageQueue, debug);
    static RadioController radioController(receiver, fc, radioControllerRates);

    static MSP_ProtoFlight msp(ahrs, fc, radioController, debug, nvs, features);
    static const MSP_Stream mspStream(msp);

    std::array<uint8_t, 128> buf;
    StreamBuf sbuf(&buf[0], sizeof(buf));
    msp.processOutCommand(MSP_FEATURE_CONFIG, sbuf);
    sbuf.reset();
    const uint32_t featuresRead = sbuf.readU32();

    TEST_ASSERT_EQUAL(features.enabledFeatures(), featuresRead);
}

void test_msp_raw_imu()
{
    static NonVolatileStorage nvs;
    static Features features;
    static MadgwickFilter sensorFusionFilter;
    static IMU_Null imu;
    static IMU_FiltersBase imuFilters;
    static AHRS ahrs(AHRS::TIMER_DRIVEN, sensorFusionFilter, imu, imuFilters);
    enum { MOTOR_COUNT = 4 };
    static Debug debug;
    static MotorMixerBase motorMixer(MOTOR_COUNT, debug);
    static ReceiverNull receiver;
    static BlackboxMessageQueue blackboxMessageQueue;
    static FlightController fc(AHRS_TASK_INTERVAL_MICROSECONDS, OUTPUT_TO_MOTORS_DENOMINATOR, ahrs, motorMixer, blackboxMessageQueue, debug);
    static RadioController radioController(receiver, fc, radioControllerRates);

    static MSP_ProtoFlight msp(ahrs, fc, radioController, debug, nvs, features);
    static MSP_Stream mspStream(msp);
    //static const MSP_Serial mspSerial(mspStream, msp);

    imu.setAccRaw({3, 5, 7});
    imu.setGyroRaw({11, 13, 17});

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    mspStream.processReceivedPacketData('M');
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData('<'); // command packet
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(1); // size
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(1, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(MSP_RAW_IMU); // command
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(103, mspStream.getCheckSum1());

    // checksum
    mspStream.processReceivedPacketData(103);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    // any character after the checksum completes the command
    mspStream.processReceivedPacketData(0);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_COMMAND_RECEIVED, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    MSP_Base::packet_t reply = mspStream.processInbuf();

    TEST_ASSERT_EQUAL(MSP_RAW_IMU, reply.cmd);
    const uint16_t accX = reply.payload.readU16();
    const uint16_t accY = reply.payload.readU16();
    const uint16_t accZ = reply.payload.readU16();
    const uint16_t gyroX = reply.payload.readU16();
    const uint16_t gyroY = reply.payload.readU16();
    const uint16_t gyroZ = reply.payload.readU16();
    const uint16_t magX = reply.payload.readU16();
    const uint16_t magY = reply.payload.readU16();
    const uint16_t magZ = reply.payload.readU16();
    TEST_ASSERT_EQUAL(3, accX);
    TEST_ASSERT_EQUAL(5, accY);
    TEST_ASSERT_EQUAL(7, accZ);
    TEST_ASSERT_EQUAL(11, gyroX);
    TEST_ASSERT_EQUAL(13, gyroY);
    TEST_ASSERT_EQUAL(17, gyroZ);
    TEST_ASSERT_EQUAL(0, magX);
    TEST_ASSERT_EQUAL(0, magY);
    TEST_ASSERT_EQUAL(0, magZ);

    reply.payload.switchToReader(); // change streambuf direction
    const MSP_Stream::packet_with_header_t pwh = mspStream.serialEncode(reply, MSP_Base::V1); // encode with MSP version 1

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(98, pwh.checksum);
    TEST_ASSERT_EQUAL(18, pwh.dataLen);
}
// NOLINTEND(misc-const-correctness)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_msp_set_failsafe_config);

    RUN_TEST(test_msp_pid_in);
    RUN_TEST(test_msp_features);
    RUN_TEST(test_msp_raw_imu);

    UNITY_END();
}
