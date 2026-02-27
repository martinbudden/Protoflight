#include "FC_Telemetry.h"

#include "FC_TelemetryData.h"
#include "FlightController.h"

#include <sv_telemetry_data.h>

#include <cstring>
#include <debug.h>
#include <motor_mixer_base.h>
#include <msp_base.h>
#if (__cplusplus >= 202002L)
#include <ranges>
#endif
#include <stream_buf_writer.h>


#if false
/*!
Packs the FlightController PID telemetry data into a TD_FC_PIDS packet. Returns the length of the packet.
*/
size_t pack_telemetry_data_PID(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, const FlightController& flightController)
{
    static_assert(static_cast<int>(TD_FC_PIDS::PID_COUNT) == static_cast<int>(FlightController::PID_COUNT));
    auto* td = reinterpret_cast<TD_FC_PIDS*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)

    td->id = id;
    td->type = TD_FC_PIDS::TYPE;
    td->len = sizeof(TD_FC_PIDS);
    td->sequence_number = static_cast<uint8_t>(sequence_number);

#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{FlightController::PID_BEGIN}, size_t{FlightController::PID_COUNT})) {
#else
    for (size_t ii = FlightController::PID_BEGIN; ii < FlightController::PID_COUNT; ++ii) {
#endif
        td->spids[ii].setpoint = flightController.get_pid_setpoint(static_cast<FlightController::pid_index_e>(ii));
        td->spids[ii].pid = flightController.get_pid_constants(static_cast<FlightController::pid_index_e>(ii));
    }

    return td->len;
}
#endif

/*!
Packs the FlightController telemetry data into a TD_FC_QUADCOPTER packet. Returns the length of the packet.
*/
size_t pack_telemetry_data_fc_quadcopter(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, const FlightController& flightController, const MotorMixerBase& motorMixer)
{
    auto* td = reinterpret_cast<TD_FC_QUADCOPTER*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)

    td->id = id;
    td->type = TD_FC_QUADCOPTER::TYPE;
    td->len = sizeof(TD_FC_QUADCOPTER);
    //td->sub_type = (TD_FC_QUADCOPTER::PID_COUNT << 4U) | TD_FC_QUADCOPTER::MOTOR_COUNT;
    td->sub_type = 0;
    td->sequence_number = static_cast<uint8_t>(sequence_number);

    //!!td->taskIntervalTicks = static_cast<uint8_t>(vehicleControllerTask.get_tick_count_delta());

    td->flags = motorMixer.motors_is_on() ? TD_FC_QUADCOPTER::MOTORS_ON_FLAG : 0x00;
    td->flags |= static_cast<uint16_t>(static_cast<uint16_t>(TD_FC_QUADCOPTER::CONTROL_MODE_MASK) & static_cast<uint16_t>(flightController.getControlMode())); //NOLINT(hicpp-signed-bitwise)

    const flight_controller_quadcopter_telemetry_t telemetryData = flightController.getTelemetryData(motorMixer);
    //!!memcpy(&td->data, &telemetryData, sizeof(flight_controller_quadcopter_telemetry_t));
    //!!td->pidErrors[TD_FC_QUADCOPTER::ROLL_RATE_DPS] = telemetryData.rollRateError;
#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{TD_FC_QUADCOPTER::MOTOR_COUNT})) {
#else
    for (size_t ii = 0; ii < TD_FC_QUADCOPTER::MOTOR_COUNT; ++ii) {
#endif
        td->data.motors[ii].power = telemetryData.motors[ii].power;
        td->data.motors[ii].rpm = telemetryData.motors[ii].rpm;
    }

    return td->len;
};

size_t pack_telemetry_data_msp(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, msp_parameter_group_t& pg, MspBase& msp, int16_t cmd_msp)
{
    (void)telemetry_data_ptr;
    (void)id;
    (void)sequence_number;
    (void)msp;
    (void)cmd_msp;

    auto* td = reinterpret_cast<TD_MSP*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    td->id = id;

    td->data.msp.header_dollar = '$';
    td->data.msp.header_m = 'M';
    td->data.msp.header_direction = '>';
    td->data.msp.payload_size = 0;
    td->data.msp.message_type = static_cast<uint8_t>(cmd_msp);

    StreamBufWriter sbuf(&td->data.msp.payload[0], sizeof(TD_MSP));
    std::array<uint8_t, 128> read_buf;
    StreamBufReader sbuf_reader(&read_buf[0], sizeof(read_buf));

    msp.process_get_set_command(pg, cmd_msp, sbuf, sbuf_reader);
    const auto payload_size = static_cast<uint8_t>(sbuf.bytes_written());
    uint8_t checksum = 0;
    for (size_t ii = 0; ii < payload_size; ++ii) {
        checksum ^= td->data.msp.payload[ii];
    }
    td->data.msp.payload_size = payload_size;
    td->data.msp.payload[payload_size] = checksum;

    return payload_size + TD_MSP::PACKET_OVERHEAD;
};

/*!
Packs the TD_Debug packet. Returns the length of the packet.
*/
size_t pack_telemetry_data_debug(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, const Debug& debug)
{
    auto* td = reinterpret_cast<TD_DEBUG*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)

    td->id = id;
    td->type = TD_DEBUG::TYPE;
    td->len = sizeof(TD_DEBUG);
    td->sub_type = 0;
    td->sequence_number = static_cast<uint8_t>(sequence_number);

    static_assert(static_cast<int>(TD_DEBUG::VALUE_COUNT) == static_cast<int>(Debug::VALUE_COUNT));
    td->data.values = debug.getValues();
    td->data.mode = debug.getMode();

    return td->len;
}
