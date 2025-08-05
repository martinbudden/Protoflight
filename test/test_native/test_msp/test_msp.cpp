#include <IMU_Filters.h> // test code won't build if this not included
#include <MSP_Protocol_Base.h>
#include <MSP_Serial.h>
#include <MSP_Stream.h>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-redundant-access-specifiers)
void test_msp_out()
{
    static MSP_Base msp;
    static const MSP_Stream mspStream(msp);

    std::array<uint8_t, 128> buf;
    StreamBuf sbuf(&buf[0], sizeof(buf));

    msp.processOutCommand(MSP_BASE_API_VERSION, sbuf, 0, nullptr);
    TEST_ASSERT_EQUAL(sizeof(buf) - 3, sbuf.bytesRemaining());
    sbuf.switchToReader();
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, sbuf.readU8());
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, sbuf.readU8());
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, sbuf.readU8());
}

void test_msp_state()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    mspStream.processReceivedPacketData('M');
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData('>'); // reply packet
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_REPLY, mspStream.getPacketType());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(3); // size
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(3, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(MSP_BASE_API_VERSION); // command
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(2, mspStream.getCheckSum1());

    // 3 bytes of data
    mspStream.processReceivedPacketData(1);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(3, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(2);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(1, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(3);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(2, mspStream.getCheckSum1());

    // checksum
    mspStream.processReceivedPacketData(2);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_COMMAND_RECEIVED, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(2, mspStream.getCheckSum1());

    // next byte puts port into idle
    mspStream.processReceivedPacketData(0);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
}

void test_msp_api_version()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

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

    mspStream.processReceivedPacketData(MSP_BASE_API_VERSION); // command
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    // checksum
    mspStream.processReceivedPacketData(3);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(3, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(3);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_COMMAND_RECEIVED, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(3, mspStream.getCheckSum1());

    MSP_Base::packet_t reply = mspStream.processInbuf();

    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION, reply.cmd);
    const uint8_t b0 = reply.payload.readU8();
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, b0);
    const uint8_t b1 = reply.payload.readU8();
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, b1);
    const uint8_t b2 = reply.payload.readU8();
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, b2);

    reply.payload.switchToReader(); // change streambuf direction
    const MSP_Stream::packet_with_header_t pwh = mspStream.serialEncode(reply, MSP_Base::V1); // encode with MSP version 1

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.dataLen);
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, *(pwh.dataPtr+2));
}

void test_msp_api_version_putchar()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    bool complete = mspStream.putChar('M', &pwh);
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());

    mspStream.putChar('<', &pwh); // command packet
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());

    mspStream.putChar(1, &pwh); // size
    TEST_ASSERT_EQUAL(1, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());

    mspStream.putChar(MSP_BASE_API_VERSION, &pwh); // command
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    const uint8_t payload = 19;
    complete = mspStream.putChar(19, &pwh); // arbitrary 1-byte payload
    TEST_ASSERT_EQUAL(payload, mspStream.getCheckSum1()); // after first put, checksum is payload
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());

    const uint8_t checkSum = 19;
    complete = mspStream.putChar(checkSum, &pwh);
    TEST_ASSERT_EQUAL(checkSum, mspStream.getCheckSum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.dataLen);
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, *(pwh.dataPtr+2));
}

void test_msp_api_version_putchar_array_stream()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    const uint8_t payloadSize = 1;
    const uint8_t payload = 19;
    const uint8_t checksum = payload; // for 1-byte payload, checksum is payload
    const std::array<uint8_t, 6> inStream = {
        'M', '<', payloadSize, MSP_BASE_API_VERSION, payload, checksum,
    };

    bool complete = mspStream.putChar(inStream[0], &pwh);
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.putChar(inStream[1], &pwh); // command packet
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.putChar(inStream[2], &pwh); // size = 1
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(1, mspStream.getCheckSum1());

    mspStream.putChar(inStream[3], &pwh); // command
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    complete = mspStream.putChar(inStream[4], &pwh); // 1-byte payload
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());
    const uint8_t checkSum = mspStream.getCheckSum1();
    TEST_ASSERT_EQUAL(inStream[4], checkSum); // after first put, checksum is payload

    complete = mspStream.putChar(inStream[5], &pwh); // checksum
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE
    TEST_ASSERT_EQUAL(inStream[5], mspStream.getCheckSum1());


    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.dataLen);
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, *(pwh.dataPtr+2));
}

void test_msp_api_version_putchar_array_stream_no_payload()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    const uint8_t payloadSize = 0;
    const uint8_t checksum = 1; // for 0-byte payload, checksum is 1
    const std::array<uint8_t, 5> inStream = {
        'M', '<', payloadSize, MSP_BASE_API_VERSION, checksum,
    };

    MSP_Stream::packet_with_header_t pwh;

    bool complete = mspStream.putChar(inStream[0], &pwh);
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());

    mspStream.putChar(inStream[1], &pwh); // command packet
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());

    mspStream.putChar(inStream[2], &pwh); // size = 1
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[3], &pwh); // command
    TEST_ASSERT_EQUAL(1, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());

    complete = mspStream.putChar(inStream[4], &pwh); // checksum
    TEST_ASSERT_EQUAL(inStream[4], mspStream.getCheckSum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.dataLen);
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, *(pwh.dataPtr+2));
}

void test_msp_api_version_putchar_array_stream_loop()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    const uint8_t payloadSize = 0;
    const uint8_t checksum = 1; // for 0-byte payload, checksum is 1
    const std::array<uint8_t, 5> inStream = {
        'M', '<', payloadSize, MSP_BASE_API_VERSION, checksum,
    };

    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.putChar(inChar, &pwh);
        if (eof) {
            break;
        }
    }

    TEST_ASSERT_EQUAL(inStream[4], mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.dataLen);
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, *(pwh.dataPtr+2));
}

class MSP_Test : public MSP_Base {
public:
    enum { MSP_SET_NAME = 11 };
public:
    virtual result_e processInCommand(int16_t cmdMSP, StreamBuf& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) override;
public:
    std::array<uint8_t, 8> _name;
};

MSP_Base::result_e MSP_Test::processInCommand(int16_t cmdMSP, StreamBuf& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn)
{
    (void)srcDesc;
    (void)postProcessFn;

    switch (cmdMSP) { // NOLINT(hicpp-multiway-paths-covered)
    case MSP_SET_NAME: {
        _name.fill(0xFF);
        size_t ii = 0;
        while (src.bytesRemaining()) {
            _name[ii++] = src.readU8();
        }
        _name[ii] = 0; // zero terminate
        break;
    }
    default:
        return RESULT_ERROR;
    }
    return RESULT_ACK;
}

void test_msp_set_name()
{
    static MSP_Test msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    const uint8_t payloadSize = 6;
    const uint8_t checksum = 30;
    const std::array<uint8_t, 13> inStream = {
        'M', '<', payloadSize, MSP_Test::MSP_SET_NAME,
        'M', 'y', 'N', 'a', 'm', 'e',
        checksum
    };
#if false
    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.putChar(inChar, &pwh);
        if (eof) {
            break;
        }
    }
#endif

    bool complete = mspStream.putChar(inStream[0], &pwh); // M
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());

    mspStream.putChar(inStream[1], &pwh); // <
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());

    mspStream.putChar(inStream[2], &pwh); // size = 6
    TEST_ASSERT_EQUAL(6, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[3], &pwh); // P1
    TEST_ASSERT_EQUAL(13, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[4], &pwh);
    TEST_ASSERT_EQUAL(64, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[5], &pwh);
    TEST_ASSERT_EQUAL(57, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[6], &pwh);
    TEST_ASSERT_EQUAL(119, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[7], &pwh);
    TEST_ASSERT_EQUAL(22, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[8], &pwh);
    TEST_ASSERT_EQUAL(123, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[9], &pwh);
    TEST_ASSERT_EQUAL(30, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());

    complete = mspStream.putChar(inStream[10], &pwh); // checksum
    TEST_ASSERT_EQUAL(30, mspStream.getCheckSum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(11, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.dataLen);

    TEST_ASSERT_EQUAL(msp._name[0], 'M');
    TEST_ASSERT_EQUAL(msp._name[1], 'y');
    TEST_ASSERT_EQUAL(msp._name[2], 'N');
    TEST_ASSERT_EQUAL(msp._name[3], 'a');
    TEST_ASSERT_EQUAL(msp._name[4], 'm');
    TEST_ASSERT_EQUAL(msp._name[5], 'e');
    TEST_ASSERT_EQUAL(msp._name[6], 0);
    TEST_ASSERT_EQUAL(msp._name[7], 0xFF);
}

void test_msp_set_name_loop()
{
    static MSP_Test msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    const uint8_t payloadSize = 6;
    const uint8_t checksum = 30;
    const std::array<uint8_t, 13> inStream = {
        'M', '<', payloadSize, MSP_Test::MSP_SET_NAME,
        'M', 'y', 'N', 'a', 'm', 'e',
        checksum
    };

    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.putChar(inChar, &pwh);
        if (eof) {
            break;
        }
    }

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(11, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.dataLen);

    TEST_ASSERT_EQUAL(msp._name[0], 'M');
    TEST_ASSERT_EQUAL(msp._name[1], 'y');
    TEST_ASSERT_EQUAL(msp._name[2], 'N');
    TEST_ASSERT_EQUAL(msp._name[3], 'a');
    TEST_ASSERT_EQUAL(msp._name[4], 'm');
    TEST_ASSERT_EQUAL(msp._name[5], 'e');
    TEST_ASSERT_EQUAL(msp._name[6], 0);
    TEST_ASSERT_EQUAL(msp._name[7], 0xFF);
}
// NOLINTEND(cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-redundant-access-specifiers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_msp_out);
    RUN_TEST(test_msp_state);
    RUN_TEST(test_msp_api_version);
    RUN_TEST(test_msp_api_version_putchar);
    RUN_TEST(test_msp_api_version_putchar_array_stream);
    RUN_TEST(test_msp_api_version_putchar_array_stream_no_payload);
    RUN_TEST(test_msp_api_version_putchar_array_stream_loop);
    RUN_TEST(test_msp_set_name);
    RUN_TEST(test_msp_set_name_loop);

    UNITY_END();
}
