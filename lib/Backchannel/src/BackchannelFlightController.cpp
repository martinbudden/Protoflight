#include "BackchannelFlightController.h"

#include "FC_Telemetry.h"
#include "FlightController.h"
#include "NonVolatileStorage.h"

//#define USE_DEBUG_PRINTF_BACKCHANNEL
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
#include <HardwareSerial.h>
#endif
#include <ahrs.h>
#if !defined(FRAMEWORK_TEST)
#include <ahrs_message_queue.h>
#endif
#include <backchannel_transceiver_base.h>
#include <debug.h>
#include <motor_mixer_base.h>
#include <receiver_base.h>
#include <receiver_telemetry.h>
#include <sv_telemetry.h>
#include <sv_telemetry_data.h>
#include <task_base.h>


BackchannelFlightController::BackchannelFlightController(BackchannelTransceiverBase& backchannelTransceiver, const uint8_t* backchannel_mac_address, const uint8_t* my_mac_address) :
    BackchannelBase(backchannelTransceiver, backchannel_mac_address, my_mac_address)
{
#if !defined(ESP_NOW_MAX_DATA_LEN)
#define ESP_NOW_MAX_DATA_LEN (250)
#endif
    static_assert(sizeof(TD_MPC) <= ESP_NOW_MAX_DATA_LEN); // 124
    static_assert(sizeof(TD_SBR_PID) <= ESP_NOW_MAX_DATA_LEN); // 96
}

bool BackchannelFlightController::packet_set_offset(backchannel_parameter_group_t& pg, const CommandPacketSetOffset& packet)
{
    xyz_t gyro_offset = pg.ahrs.get_gyro_offset_mapped();
    xyz_t acc_offset = pg.ahrs.get_acc_offset_mapped();

#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
    //Serial.printf("    packet_set_offset itype:%d, len:%d value:%d, type:%d\r\n", packet.type, packet.len, packet.value, packet.type);
#endif
    switch (packet.set_type) {
    case CommandPacketSetOffset::SET_GYRO_OFFSET_X:
        gyro_offset.x = packet.value;
        pg.ahrs.set_gyro_offset_mapped(gyro_offset);
        break;
    case CommandPacketSetOffset::SET_GYRO_OFFSET_Y:
        gyro_offset.y = packet.value;
        pg.ahrs.set_gyro_offset_mapped(gyro_offset);
        break;
    case CommandPacketSetOffset::SET_GYRO_OFFSET_Z:
        gyro_offset.z = packet.value;
        pg.ahrs.set_gyro_offset_mapped(gyro_offset);
        break;
    case CommandPacketSetOffset::SET_ACC_OFFSET_X:
        acc_offset.x = packet.value;
        pg.ahrs.set_acc_offset_mapped(acc_offset);
        break;
    case CommandPacketSetOffset::SET_ACC_OFFSET_Y:
        acc_offset.y = packet.value;
        pg.ahrs.set_acc_offset_mapped(acc_offset);
        break;
    case CommandPacketSetOffset::SET_ACC_OFFSET_Z:
        acc_offset.z = packet.value;
        pg.ahrs.set_acc_offset_mapped(acc_offset);
        break;
    case CommandPacketSetOffset::SAVE_GYRO_OFFSET: {
        pg.nvs.store_gyro_offset(pg.ahrs.get_gyro_offset());
        break;
    }
    case CommandPacketSetOffset::SAVE_ACC_OFFSET: {
        pg.nvs.store_acc_offset(pg.ahrs.get_acc_offset());
        break;
    }
    default:
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
        Serial.printf("Backchannel::packet_set_offset invalid item_index:%d\r\n", packet.set_type);
#endif
        return false;
    }
    return true;
}

bool BackchannelFlightController::packet_control(backchannel_parameter_group_t& pg, const CommandPacketControl& packet)
{
    //Serial.printf("Control packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);
    switch (packet.control) {
    case CommandPacketControl::MOTORS_SWITCH_OFF:
        pg.flight_controller.motorsSwitchOff(pg.motor_mixer);
        return true;
    case CommandPacketControl::MOTORS_SWITCH_ON:
        pg.flight_controller.motorsSwitchOn(pg.motor_mixer);
        return true;
    case CommandPacketControl::RESET:
        pg.motor_mixer.reset_all_encoders();
        return true;
    case CommandPacketControl::SET_MODE:
        pg.flight_controller.setControlMode(static_cast<FlightController::control_mode_e>(packet.value));
        return true;
    case CommandPacketControl::SET_PID_PROFILE:
        return false; // not implemented
    case CommandPacketControl::SET_RATES_PROFILE:
        return false; // not implemented
    case CommandPacketControl::SET_DEBUG_MODE:
        pg.debug.setMode(static_cast<debug_mode_e>(packet.value));
        return true;
    default:
        // do nothing
        break;
    } // end switch
    return false;
}

bool BackchannelFlightController::packet_set_pid(backchannel_parameter_group_t& pg, const CommandPacketSetPid& packet)
{
    //Serial.printf("SetPID packet type:%d, len:%d, pid_index:%d set_type:%d value:%3d f0:%f\r\n", packet.type, packet.len, packet.pid_index, packet.set_type, packet.value, packet.f0);

    static_assert(static_cast<int>(TD_PID::MAX_PID_COUNT) >= static_cast<int>(FlightController::PID_COUNT));

    const auto pid_index = static_cast<FlightController::pid_index_e>(packet.pid_index);
    if (pid_index >= FlightController::PID_COUNT) {
        //Serial.printf("Backchannel::packet_set_pid invalid pid_index:%d\r\n", packet.pid_index);
        return false;
    }

    bool transmit = false;
    switch (packet.set_type) {
    case CommandPacketSetPid::SET_P:
        pg.flight_controller.set_pid_p_msp(pid_index, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPid::SET_I:
        pg.flight_controller.set_pid_i_msp(pid_index, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPid::SET_D:
        pg.flight_controller.set_pid_d_msp(pid_index, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPid::SET_S:
        pg.flight_controller.set_pid_s_msp(pid_index, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPid::SET_K:
        pg.flight_controller.set_pid_k_msp(pid_index, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPid::SET_PD: {
        // Set P and change D to preserve P/D ratio
        pg.flight_controller.set_pid_pd_msp(pid_index, packet.value);
        transmit = true;
        break;
    }
    case CommandPacketSetPid::SAVE_P: // NOLINT(bugprone-branch-clone) false positive
        [[fallthrough]];
    case CommandPacketSetPid::SAVE_I:
        [[fallthrough]];
    case CommandPacketSetPid::SAVE_D:
        [[fallthrough]];
    case CommandPacketSetPid::SAVE_S:
        [[fallthrough]];
    case CommandPacketSetPid::SAVE_K:
        [[fallthrough]];
    case CommandPacketSetPid::SAVE_PD:
        //Serial.printf("Saved PID packetType:%d pid_index:%d set_type:%d\r\n", packet.type, packet.pid_index, packet.set_type);
        // Currently we don't save individual PID constants: if any save request is received we save all the PID constants.
        pg.nvs.store_pid(pg.flight_controller.get_pid_constants(pid_index), pid_index);
        return true;
    case CommandPacketSetPid::RESET_PID:
        pg.nvs.reset_pid(pid_index);
        return true;
    default:
        //Serial.printf("Backchannel::packet_set_pid invalid set_type:%d\r\n", packet.pid_index);
        break;
    }

    if (transmit) {
        // send back the new data for display
        const size_t len = pack_telemetry_data_pid(
            _transmit_data_buffer_ptr,
            _telemetry_id,
            _sequence_number,
            pg.flight_controller,
            pg.nvs.get_current_pid_profile_index(),
            pg.flight_controller.getControlMode(),
            0.0F,
            0.0F
        );
        send_data(_transmit_data_buffer_ptr, len);
        return true;
    }
    return false;
}

bool BackchannelFlightController::send_packet(backchannel_parameter_group_t& pg, uint8_t sub_command)
{
    switch (_request_type) {
    case CommandPacketRequestData::NO_REQUEST: {
        return false;
    }
    case CommandPacketRequestData::REQUEST_STOP_SENDING_DATA: {
        // send a minimal packet so the client can reset its screen
        const size_t len = pack_telemetry_data_minimal(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number);
        send_data(_transmit_data_buffer_ptr, len);
        // set _request_type to NO_REQUEST so no further data sent
        _request_type = CommandPacketRequestData::NO_REQUEST;
        return true;
    }
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_DATA: {
        const size_t len = pack_telemetry_data_task_intervals(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number,
            *pg.ahrs.get_task(),
            *pg.flight_controller.get_task(),
            pg.main_task ?  pg.main_task->get_tick_count_delta() : 0,
            _backchannel_transceiver.get_tick_count_delta_and_reset()
        );
        //Serial.printf("tiLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        return true;
    }
#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_EXTENDED_DATA: {
        // intercept AHRS_DATA request add additional timing information
        const size_t len = pack_telemetry_data_task_intervals_extended(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number,
            pg.ahrs,
            pg.flight_controller,
            pg.main_task ? pg.main_task->get_tick_count_delta() : 0,
            _backchannel_transceiver.get_tick_count_delta_and_reset(),
            static_cast<uint32_t>(pg.receiver.get_dropped_packet_count_delta())
        );
        // use empty slots to add additional time checks
        // timings [0-3] are set in AHRS readIMUandUpdateOrientation(), timings [4-7] are free to use
        auto* td = reinterpret_cast<TD_TASK_INTERVALS_EXTENDED*>(_transmit_data_buffer_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        td->data.ahrsTimeChecksMicroseconds[4] = pg.flight_controller.getTimeChecksMicroseconds(0);
        td->data.ahrsTimeChecksMicroseconds[5] = pg.flight_controller.getTimeChecksMicroseconds(1);
        td->data.ahrsTimeChecksMicroseconds[6] = pg.flight_controller.getTimeChecksMicroseconds(2);
        td->data.ahrsTimeChecksMicroseconds[7] = pg.flight_controller.getTimeChecksMicroseconds(3);
        //Serial.printf("tiLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        return true;
    }
#endif
#if !defined(FRAMEWORK_TEST)
    case CommandPacketRequestData::REQUEST_AHRS_DATA: {
        // intercept AHRS_DATA request to replace roll and pitch values
        ahrs_data_t ahrsData {};
        pg.ahrs_message_queue.PEEK_AHRS_DATA(ahrsData);
        const size_t len = pack_telemetry_data_ahrs(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number, pg.ahrs, ahrsData);
        auto* td = reinterpret_cast<TD_AHRS*>(_transmit_data_buffer_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        // convert from ENU to NED
        const Quaternion orientationENU = ahrsData.orientation;
        td->data.roll = -orientationENU.calculate_pitch_degrees(),
        td->data.pitch = orientationENU.calculate_roll_degrees(),
        td->data.yaw = orientationENU.calculate_yaw_degrees(),
        send_data(_transmit_data_buffer_ptr, len);
        return true;
    }
#endif
    case CommandPacketRequestData::REQUEST_RECEIVER_DATA: {
        const size_t len = pack_telemetry_data_receiver(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number, pg.receiver);
        //Serial.printf("receiverLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        return true;
    }
    case CommandPacketRequestData::REQUEST_PID_DATA: {
        const size_t len = pack_telemetry_data_pid(
            _transmit_data_buffer_ptr,
            _telemetry_id,
            _sequence_number,
            pg.flight_controller,
            pg.nvs.get_current_pid_profile_index(),
            pg.flight_controller.getControlMode(),
            0.0F,
            0.0F
        );
        //Serial.printf("pidLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        // reset _request_type to NO_REQUEST, since REQUEST_PID_DATA is a one shot, as response to keypress
        _request_type = CommandPacketRequestData::NO_REQUEST;
        return true;
    }
    case CommandPacketRequestData::REQUEST_PID_ERROR_DATA: {
        const size_t len = pack_telemetry_data_pid_outputs(
            _transmit_data_buffer_ptr,
            _telemetry_id,
            _sequence_number,
            pg.flight_controller,
            pg.nvs.get_current_pid_profile_index(),
            pg.flight_controller.getControlMode()
        );
        //Serial.printf("vcLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        return true;
    }
    case CommandPacketRequestData::REQUEST_VEHICLE_CONTROLLER_DATA: {
        const size_t len = pack_telemetry_data_fc_quadcopter(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number, pg.flight_controller, pg.motor_mixer);
        //Serial.printf("vcLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        return true;
    }
    case CommandPacketRequestData::REQUEST_DEBUG_DATA: {
        const size_t len = pack_telemetry_data_debug(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number, pg.debug);
        //Serial.printf("debugLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        return true;
    }
    case CommandPacketRequestData::REQUEST_MSP_DATA: {
        if (pg.msp && pg.msp_parameter_group) {
            const size_t len = pack_telemetry_data_msp(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number, *pg.msp_parameter_group, *pg.msp, sub_command);
            if (len <= ESP_NOW_MAX_DATA_LEN) {
                send_data(_transmit_data_buffer_ptr, len);
            }
        }
        return true;
    }
    default:
        return false;
    } // end switch
}
