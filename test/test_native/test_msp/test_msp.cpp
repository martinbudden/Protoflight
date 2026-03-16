#include <msp_protocol.h>
#include <msp_serial.h>
#include <msp_stream.h>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

struct msp_context_t {};

// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,misc-const-correctness,misc-non-private-member-variables-in-classes,readability-redundant-access-specifiers)
void test_msp_out()
{
    static MspBase msp;
    static const MspStream msp_stream(msp);
    static msp_context_t ctx {};


    std::array<uint8_t, 128> write_buf;
    StreamBufWriter sbuf_writer(&write_buf[0], sizeof(write_buf));
    std::array<uint8_t, 128> read_buf;
    StreamBufReader sbuf_reader(&read_buf[0], sizeof(read_buf));

    msp.process_write_command(ctx, MSP_API_VERSION, sbuf_writer, sbuf_reader);
    TEST_ASSERT_EQUAL(sizeof(write_buf) - 3, sbuf_writer.bytes_remaining());
    sbuf_writer.switch_to_reader();
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, sbuf_writer.read_u8());
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, sbuf_writer.read_u8());
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, sbuf_writer.read_u8());
}

void test_msp_state()
{
    static MspBase msp;
    static MspStream msp_stream(msp);

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream.process_received_packet_data('M');
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    msp_stream.process_received_packet_data('>'); // reply packet
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_REPLY, msp_stream.get_packet_type());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    msp_stream.process_received_packet_data(3); // size
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(3, msp_stream.get_checksum1());

    msp_stream.process_received_packet_data(MSP_API_VERSION); // command
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(2, msp_stream.get_checksum1());

    // 3 bytes of data
    msp_stream.process_received_packet_data(1);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(3, msp_stream.get_checksum1());

    msp_stream.process_received_packet_data(2);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(1, msp_stream.get_checksum1());

    msp_stream.process_received_packet_data(3);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(2, msp_stream.get_checksum1());

    // checksum
    msp_stream.process_received_packet_data(2);
    TEST_ASSERT_EQUAL(MSP_COMMAND_RECEIVED, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(2, msp_stream.get_checksum1());

    // next byte puts port into idle
    msp_stream.process_received_packet_data(0);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
}

void test_msp_api_version()
{
    static MspBase msp;
    static MspStream msp_stream(msp);
    static msp_context_t ctx {};

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream.process_received_packet_data('M');
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    msp_stream.process_received_packet_data('<'); // command packet
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, msp_stream.get_packet_type());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    msp_stream.process_received_packet_data(1); // size
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(1, msp_stream.get_checksum1());

    msp_stream.process_received_packet_data(MSP_API_VERSION); // command
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    // checksum
    msp_stream.process_received_packet_data(3);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(3, msp_stream.get_checksum1());

    msp_stream.process_received_packet_data(3);
    TEST_ASSERT_EQUAL(MSP_COMMAND_RECEIVED, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(3, msp_stream.get_checksum1());

    msp_const_packet_t reply = msp_stream.process_in_buf(ctx);

    TEST_ASSERT_EQUAL(MSP_API_VERSION, reply.cmd);
    const uint8_t b0 = reply.payload.read_u8();
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, b0);
    const uint8_t b1 = reply.payload.read_u8();
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, b1);
    const uint8_t b2 = reply.payload.read_u8();
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, b2);

    reply.payload.switch_to_reader(); // change StreamBufWriter direction
    const msp_stream_packet_with_header_t pwh = msp_stream.serial_encode(reply, MSP_V1); // encode with MSP version 1

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
    static MspStream msp_stream(msp);
    static msp_context_t ctx {};

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    bool complete = msp_stream.put_char(ctx, 'M', &pwh);
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, '<', &pwh); // command packet
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, msp_stream.get_packet_type());

    msp_stream.put_char(ctx, 1, &pwh); // size
    TEST_ASSERT_EQUAL(1, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, MSP_API_VERSION, &pwh); // command
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    const uint8_t payload = 19;
    complete = msp_stream.put_char(ctx, 19, &pwh); // arbitrary 1-byte payload
    TEST_ASSERT_EQUAL(payload, msp_stream.get_checksum1()); // after first put, checksum is payload
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());

    const uint8_t checkSum = 19;
    complete = msp_stream.put_char(ctx, checkSum, &pwh);
    TEST_ASSERT_EQUAL(checkSum, msp_stream.get_checksum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

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
    static MspStream msp_stream(msp);
    static msp_context_t ctx {};

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payload_size = 1;
    const uint8_t payload = 19;
    const uint8_t checksum = payload; // for 1-byte payload, checksum is payload
    const std::array<uint8_t, 6> inStream = {
        'M', '<', payload_size, MSP_API_VERSION, payload, checksum,
    };

    bool complete = msp_stream.put_char(ctx, inStream[0], &pwh);
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    msp_stream.put_char(ctx, inStream[1], &pwh); // command packet
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, msp_stream.get_packet_type());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    msp_stream.put_char(ctx, inStream[2], &pwh); // size = 1
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(1, msp_stream.get_checksum1());

    msp_stream.put_char(ctx, inStream[3], &pwh); // command
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    complete = msp_stream.put_char(ctx, inStream[4], &pwh); // 1-byte payload
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());
    const uint8_t checkSum = msp_stream.get_checksum1();
    TEST_ASSERT_EQUAL(inStream[4], checkSum); // after first put, checksum is payload

    complete = msp_stream.put_char(ctx, inStream[5], &pwh); // checksum
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE
    TEST_ASSERT_EQUAL(inStream[5], msp_stream.get_checksum1());


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
    static MspStream msp_stream(msp);
    static msp_context_t ctx {};

    msp_stream.set_packet_state(MSP_IDLE);

    const uint8_t payload_size = 0;
    const uint8_t checksum = 1; // for 0-byte payload, checksum is 1
    const std::array<uint8_t, 5> inStream = {
        'M', '<', payload_size, MSP_API_VERSION, checksum,
    };

    msp_stream_packet_with_header_t pwh;

    bool complete = msp_stream.put_char(ctx, inStream[0], &pwh);
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, inStream[1], &pwh); // command packet
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, msp_stream.get_packet_type());

    msp_stream.put_char(ctx, inStream[2], &pwh); // size = 1
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, inStream[3], &pwh); // command
    TEST_ASSERT_EQUAL(1, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());

    complete = msp_stream.put_char(ctx, inStream[4], &pwh); // checksum
    TEST_ASSERT_EQUAL(inStream[4], msp_stream.get_checksum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

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
    static MspStream msp_stream(msp);
    static msp_context_t ctx {};

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payload_size = 0;
    const uint8_t checksum = 1; // for 0-byte payload, checksum is 1
    const std::array<uint8_t, 5> inStream = {
        'M', '<', payload_size, MSP_API_VERSION, checksum,
    };

    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = msp_stream.put_char(ctx, inChar, &pwh);
        if (eof) {
            break;
        }
    }

    TEST_ASSERT_EQUAL(inStream[4], msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

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
    msp_result_e process_read_command(msp_context_t& ctx, int16_t cmd_msp, StreamBufReader& src) override;
public:
    std::array<uint8_t, 8> _name;
};

msp_result_e MSP_Test::process_read_command(msp_context_t& ctx, int16_t cmd_msp, StreamBufReader& src)
{
    (void)ctx;

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
    static MspStream msp_stream(msp);
    static msp_context_t ctx {};

    msp_stream.set_packet_state(MSP_IDLE);

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
        const bool eof = msp_stream.put_char(ctx, inChar, &pwh);
        if (eof) {
            break;
        }
    }
#endif

    bool complete = msp_stream.put_char(ctx, inStream[0], &pwh); // M
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, inStream[1], &pwh); // <
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, msp_stream.get_packet_type());

    msp_stream.put_char(ctx, inStream[2], &pwh); // size = 6
    TEST_ASSERT_EQUAL(6, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, inStream[3], &pwh); // P1
    TEST_ASSERT_EQUAL(13, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, inStream[4], &pwh);
    TEST_ASSERT_EQUAL(64, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, inStream[5], &pwh);
    TEST_ASSERT_EQUAL(57, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, inStream[6], &pwh);
    TEST_ASSERT_EQUAL(119, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, inStream[7], &pwh);
    TEST_ASSERT_EQUAL(22, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, inStream[8], &pwh);
    TEST_ASSERT_EQUAL(123, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(ctx, inStream[9], &pwh);
    TEST_ASSERT_EQUAL(30, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());

    complete = msp_stream.put_char(ctx, inStream[10], &pwh); // checksum
    TEST_ASSERT_EQUAL(30, msp_stream.get_checksum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

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
    static MspStream msp_stream(msp);
    static msp_context_t ctx {};

    msp_stream.set_packet_state(MSP_IDLE);

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
        const bool eof = msp_stream.put_char(ctx, inChar, &pwh);
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
