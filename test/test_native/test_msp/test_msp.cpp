#include <msp_protocol.h>
#include <msp_serial.h>
#include <msp_stream.h>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

struct msp_parameter_group_t {};

// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,misc-const-correctness,misc-non-private-member-variables-in-classes,readability-redundant-access-specifiers)
void test_msp_out()
{
    static MspBase msp;
    static const MspStream mspStream(msp);
    static msp_parameter_group_t pg {};


    std::array<uint8_t, 128> write_buf;
    StreamBufWriter sbuf_writer(&write_buf[0], sizeof(write_buf));
    std::array<uint8_t, 128> read_buf;
    StreamBufReader sbuf_reader(&read_buf[0], sizeof(read_buf));

    msp.process_get_set_command(pg, MSP_API_VERSION, sbuf_writer, sbuf_reader);
    TEST_ASSERT_EQUAL(sizeof(write_buf) - 3, sbuf_writer.bytes_remaining());
    sbuf_writer.switch_to_reader();
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, sbuf_writer.read_u8());
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, sbuf_writer.read_u8());
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, sbuf_writer.read_u8());
}

void test_msp_state()
{
    static MspBase msp;
    static MspStream mspStream(msp);

    mspStream.set_packet_state(MSP_IDLE);

    mspStream.process_received_packet_data('M');
    TEST_ASSERT_EQUAL(MSP_HEADER_M, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());

    mspStream.process_received_packet_data('>'); // reply packet
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_REPLY, mspStream.get_packet_type());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());

    mspStream.process_received_packet_data(3); // size
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(3, mspStream.get_checksum1());

    mspStream.process_received_packet_data(MSP_API_VERSION); // command
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(2, mspStream.get_checksum1());

    // 3 bytes of data
    mspStream.process_received_packet_data(1);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(3, mspStream.get_checksum1());

    mspStream.process_received_packet_data(2);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(1, mspStream.get_checksum1());

    mspStream.process_received_packet_data(3);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(2, mspStream.get_checksum1());

    // checksum
    mspStream.process_received_packet_data(2);
    TEST_ASSERT_EQUAL(MSP_COMMAND_RECEIVED, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(2, mspStream.get_checksum1());

    // next byte puts port into idle
    mspStream.process_received_packet_data(0);
    TEST_ASSERT_EQUAL(MSP_IDLE, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
}

void test_msp_api_version()
{
    static MspBase msp;
    static MspStream mspStream(msp);
    static msp_parameter_group_t pg {};

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

    mspStream.process_received_packet_data(MSP_API_VERSION); // command
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());

    // checksum
    mspStream.process_received_packet_data(3);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(3, mspStream.get_checksum1());

    mspStream.process_received_packet_data(3);
    TEST_ASSERT_EQUAL(MSP_COMMAND_RECEIVED, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(3, mspStream.get_checksum1());

    msp_const_packet_t reply = mspStream.process_in_buf(pg);

    TEST_ASSERT_EQUAL(MSP_API_VERSION, reply.cmd);
    const uint8_t b0 = reply.payload.read_u8();
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, b0);
    const uint8_t b1 = reply.payload.read_u8();
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, b1);
    const uint8_t b2 = reply.payload.read_u8();
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, b2);

    reply.payload.switch_to_reader(); // change StreamBufWriter direction
    const msp_stream_packet_with_header_t pwh = mspStream.serial_encode(reply, MSP_V1); // encode with MSP version 1

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.data_len);
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, *pwh.data_ptr);
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, *(pwh.data_ptr+1));
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, *(pwh.data_ptr+2));
}

void test_msp_api_version_putchar()
{
    static MspBase msp;
    static MspStream mspStream(msp);
    static msp_parameter_group_t pg {};

    mspStream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    bool complete = mspStream.put_char(pg, 'M', &pwh);
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, mspStream.get_packet_state());

    mspStream.put_char(pg, '<', &pwh); // command packet
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, mspStream.get_packet_type());

    mspStream.put_char(pg, 1, &pwh); // size
    TEST_ASSERT_EQUAL(1, mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());

    mspStream.put_char(pg, MSP_API_VERSION, &pwh); // command
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    const uint8_t payload = 19;
    complete = mspStream.put_char(pg, 19, &pwh); // arbitrary 1-byte payload
    TEST_ASSERT_EQUAL(payload, mspStream.get_checksum1()); // after first put, checksum is payload
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, mspStream.get_packet_state());

    const uint8_t checkSum = 19;
    complete = mspStream.put_char(pg, checkSum, &pwh);
    TEST_ASSERT_EQUAL(checkSum, mspStream.get_checksum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, mspStream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.data_len);
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, *pwh.data_ptr);
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, *(pwh.data_ptr+1));
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, *(pwh.data_ptr+2));
}

void test_msp_api_version_putchar_array_stream()
{
    static MspBase msp;
    static MspStream mspStream(msp);
    static msp_parameter_group_t pg {};

    mspStream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payload_size = 1;
    const uint8_t payload = 19;
    const uint8_t checksum = payload; // for 1-byte payload, checksum is payload
    const std::array<uint8_t, 6> inStream = {
        'M', '<', payload_size, MSP_API_VERSION, payload, checksum,
    };

    bool complete = mspStream.put_char(pg, inStream[0], &pwh);
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());

    mspStream.put_char(pg, inStream[1], &pwh); // command packet
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, mspStream.get_packet_type());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());

    mspStream.put_char(pg, inStream[2], &pwh); // size = 1
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(1, mspStream.get_checksum1());

    mspStream.put_char(pg, inStream[3], &pwh); // command
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());

    complete = mspStream.put_char(pg, inStream[4], &pwh); // 1-byte payload
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, mspStream.get_packet_state());
    const uint8_t checkSum = mspStream.get_checksum1();
    TEST_ASSERT_EQUAL(inStream[4], checkSum); // after first put, checksum is payload

    complete = mspStream.put_char(pg, inStream[5], &pwh); // checksum
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, mspStream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE
    TEST_ASSERT_EQUAL(inStream[5], mspStream.get_checksum1());


    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.data_len);
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, *pwh.data_ptr);
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, *(pwh.data_ptr+1));
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, *(pwh.data_ptr+2));
}

void test_msp_api_version_putchar_array_stream_no_payload()
{
    static MspBase msp;
    static MspStream mspStream(msp);
    static msp_parameter_group_t pg {};

    mspStream.set_packet_state(MSP_IDLE);

    const uint8_t payload_size = 0;
    const uint8_t checksum = 1; // for 0-byte payload, checksum is 1
    const std::array<uint8_t, 5> inStream = {
        'M', '<', payload_size, MSP_API_VERSION, checksum,
    };

    msp_stream_packet_with_header_t pwh;

    bool complete = mspStream.put_char(pg, inStream[0], &pwh);
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, mspStream.get_packet_state());

    mspStream.put_char(pg, inStream[1], &pwh); // command packet
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, mspStream.get_packet_type());

    mspStream.put_char(pg, inStream[2], &pwh); // size = 1
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());

    mspStream.put_char(pg, inStream[3], &pwh); // command
    TEST_ASSERT_EQUAL(1, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, mspStream.get_packet_state());

    complete = mspStream.put_char(pg, inStream[4], &pwh); // checksum
    TEST_ASSERT_EQUAL(inStream[4], mspStream.get_checksum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, mspStream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.data_len);
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, *pwh.data_ptr);
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, *(pwh.data_ptr+1));
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, *(pwh.data_ptr+2));
}

void test_msp_api_version_putchar_array_stream_loop()
{
    static MspBase msp;
    static MspStream mspStream(msp);
    static msp_parameter_group_t pg {};

    mspStream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payload_size = 0;
    const uint8_t checksum = 1; // for 0-byte payload, checksum is 1
    const std::array<uint8_t, 5> inStream = {
        'M', '<', payload_size, MSP_API_VERSION, checksum,
    };

    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.put_char(pg, inChar, &pwh);
        if (eof) {
            break;
        }
    }

    TEST_ASSERT_EQUAL(inStream[4], mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_IDLE, mspStream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.data_len);
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, *pwh.data_ptr);
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, *(pwh.data_ptr+1));
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, *(pwh.data_ptr+2));
}

class MSP_Test : public MspBase {
public:
    enum { MSP_SET_NAME = 11 };
public:
    msp_result_e process_set_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufReader& src) override;
public:
    std::array<uint8_t, 8> _name;
};

msp_result_e MSP_Test::process_set_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufReader& src)
{
    (void)pg;

    switch (cmd_msp) { // NOLINT(hicpp-multiway-paths-covered)
    case MSP_SET_NAME: {
        _name.fill(0xFF);
        size_t ii = 0;
        while (src.bytes_remaining()) {
            _name[ii++] = src.read_u8();
        }
        _name[ii] = 0; // zero terminate
        break;
    }
    default:
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

void test_msp_set_name()
{
    static MSP_Test msp;
    static MspStream mspStream(msp);
    static msp_parameter_group_t pg {};

    mspStream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payload_size = 6;
    const uint8_t checksum = 30;
    const std::array<uint8_t, 13> inStream = {
        'M', '<', payload_size, MSP_Test::MSP_SET_NAME,
        'M', 'y', 'N', 'a', 'm', 'e',
        checksum
    };
#if false
    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.put_char(pg, inChar, &pwh);
        if (eof) {
            break;
        }
    }
#endif

    bool complete = mspStream.put_char(pg, inStream[0], &pwh); // M
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, mspStream.get_packet_state());

    mspStream.put_char(pg, inStream[1], &pwh); // <
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, mspStream.get_packet_type());

    mspStream.put_char(pg, inStream[2], &pwh); // size = 6
    TEST_ASSERT_EQUAL(6, mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());

    mspStream.put_char(pg, inStream[3], &pwh); // P1
    TEST_ASSERT_EQUAL(13, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(pg, inStream[4], &pwh);
    TEST_ASSERT_EQUAL(64, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(pg, inStream[5], &pwh);
    TEST_ASSERT_EQUAL(57, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(pg, inStream[6], &pwh);
    TEST_ASSERT_EQUAL(119, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(pg, inStream[7], &pwh);
    TEST_ASSERT_EQUAL(22, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(pg, inStream[8], &pwh);
    TEST_ASSERT_EQUAL(123, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(pg, inStream[9], &pwh);
    TEST_ASSERT_EQUAL(30, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, mspStream.get_packet_state());

    complete = mspStream.put_char(pg, inStream[10], &pwh); // checksum
    TEST_ASSERT_EQUAL(30, mspStream.get_checksum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, mspStream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(11, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.data_len);

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
    static MspStream mspStream(msp);
    static msp_parameter_group_t pg {};

    mspStream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payload_size = 6;
    const uint8_t checksum = 30;
    const std::array<uint8_t, 13> inStream = {
        'M', '<', payload_size, MSP_Test::MSP_SET_NAME,
        'M', 'y', 'N', 'a', 'm', 'e',
        checksum
    };

    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.put_char(pg, inChar, &pwh);
        if (eof) {
            break;
        }
    }

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(11, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.data_len);

    TEST_ASSERT_EQUAL(msp._name[0], 'M');
    TEST_ASSERT_EQUAL(msp._name[1], 'y');
    TEST_ASSERT_EQUAL(msp._name[2], 'N');
    TEST_ASSERT_EQUAL(msp._name[3], 'a');
    TEST_ASSERT_EQUAL(msp._name[4], 'm');
    TEST_ASSERT_EQUAL(msp._name[5], 'e');
    TEST_ASSERT_EQUAL(msp._name[6], 0);
    TEST_ASSERT_EQUAL(msp._name[7], 0xFF);
}
// NOLINTEND(cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,misc-const-correctness,misc-non-private-member-variables-in-classes,readability-redundant-access-specifiers)

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
