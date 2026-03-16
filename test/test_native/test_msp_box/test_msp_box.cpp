#include "msp_box.h"
#include <msp_protocol.h>
#include <msp_serial.h>
#include <msp_stream.h>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

struct msp_context_t {};

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)
void test_msp_box()
{
    enum { BOX_HORIZON_PERMANENT = 2 };

    const MspBox::box_t* box = MspBox::find_box_by_permanent_id(BOX_HORIZON_PERMANENT);
    TEST_ASSERT_FALSE(box == nullptr);
    TEST_ASSERT_EQUAL(MspBox::BOX_HORIZON, box->id);

    enum { BOX_GPS_RESCUE_PERMANENT = 46 };
    box = MspBox::find_box_by_permanent_id(BOX_GPS_RESCUE_PERMANENT);
    TEST_ASSERT_FALSE(box == nullptr);
    TEST_ASSERT_EQUAL(MspBox::BOX_GPS_RESCUE, box->id);


    MspBox msp_box {};

    TEST_ASSERT_EQUAL(false, msp_box.get_active_box_id(MspBox::BOX_ARM));
    msp_box.set_active_box_id(MspBox::BOX_ARM);
    TEST_ASSERT_EQUAL(true, msp_box.get_active_box_id(MspBox::BOX_ARM));
    msp_box.reset_active_box_id(MspBox::BOX_ARM);
    TEST_ASSERT_EQUAL(false, msp_box.get_active_box_id(MspBox::BOX_ARM));

    msp_box.set_active_box_id(MspBox::BOX_PREARM);
    TEST_ASSERT_EQUAL(true, msp_box.get_active_box_id(MspBox::BOX_PREARM));

    msp_box.set_active_box_id(MspBox::BOX_AIRMODE);
    TEST_ASSERT_EQUAL(true, msp_box.get_active_box_id(MspBox::BOX_AIRMODE));

    msp_box.set_active_box_id(MspBox::BOX_ANTIGRAVITY);
    TEST_ASSERT_EQUAL(true, msp_box.get_active_box_id(MspBox::BOX_ANTIGRAVITY));

    msp_box.set_active_box_id(MspBox::BOX_ANGLE);
    TEST_ASSERT_EQUAL(true, msp_box.get_active_box_id(MspBox::BOX_ANGLE));

    msp_box.set_active_box_id(MspBox::BOX_HORIZON);
    TEST_ASSERT_EQUAL(true, msp_box.get_active_box_id(MspBox::BOX_HORIZON));

    msp_box.set_active_box_id(MspBox::BOX_ALTITUDE_HOLD);
    TEST_ASSERT_EQUAL(true, msp_box.get_active_box_id(MspBox::BOX_ALTITUDE_HOLD));

    msp_box.set_active_box_id(MspBox::BOX_HEADFREE);
    TEST_ASSERT_EQUAL(true, msp_box.get_active_box_id(MspBox::BOX_HEADFREE));

    msp_box.set_active_box_id(MspBox::BOX_HEADADJ);
    TEST_ASSERT_EQUAL(true, msp_box.get_active_box_id(MspBox::BOX_HEADADJ));
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

void test_get_msp_base_api_version()
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

    reply.payload.switch_to_reader(); // change streambuf direction
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

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_msp_box);
    RUN_TEST(test_msp_state);
    RUN_TEST(test_get_msp_base_api_version);

    UNITY_END();
}
