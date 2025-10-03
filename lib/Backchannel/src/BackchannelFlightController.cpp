#include "BackchannelFlightController.h"

#include "FC_Telemetry.h"

#include <AHRS.h>
#include <Debug.h>
#include <NonVolatileStorage.h>
#include <ReceiverBase.h>
#include <SV_Telemetry.h>
#include <SV_TelemetryData.h>


BackchannelFlightController::BackchannelFlightController(
        BackchannelTransceiverBase& backchannelTransceiver,
        const uint8_t* backchannelMacAddress,
        const uint8_t* myMacAddress,
        FlightController& flightController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        const TaskBase* mainTask,
        NonVolatileStorage& nonVolatileStorage
    ) :
    BackchannelStabilizedVehicle(
        backchannelTransceiver,
        backchannelMacAddress,
        myMacAddress,
        flightController,
        ahrs,
        receiver,
        mainTask
    ),
    _flightController(flightController),
    _nonVolatileStorage(nonVolatileStorage)
{
#if !defined(ESP_NOW_MAX_DATA_LEN)
#define ESP_NOW_MAX_DATA_LEN (250)
#endif
    static_assert(sizeof(TD_MPC) <= ESP_NOW_MAX_DATA_LEN); // 124
    static_assert(sizeof(TD_SBR_PID) <= ESP_NOW_MAX_DATA_LEN); // 96
}

bool BackchannelFlightController::packetSetOffset(const CommandPacketSetOffset& packet)
{
    if (BackchannelStabilizedVehicle::packetSetOffset(packet)) {
        return true;
    }

    switch (packet.setType) {
    case CommandPacketSetOffset::SAVE_GYRO_OFFSET: {
        const IMU_Base::xyz_int32_t gyroOffset = _ahrs.getGyroOffset();
        _nonVolatileStorage.storeGyroOffset(gyroOffset.x, gyroOffset.y, gyroOffset.z);
        break;
    }
    case CommandPacketSetOffset::SAVE_ACC_OFFSET: {
        const IMU_Base::xyz_int32_t accOffset = _ahrs.getAccOffset();
        _nonVolatileStorage.storeAccOffset(accOffset.x, accOffset.y, accOffset.z);
        break;
    }
    default:
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
        Serial.printf("Backchannel::packetSetOffset invalid itemIndex:%d\r\n", packet.setType);
#endif
        return false;
    }

    return true;
}

bool BackchannelFlightController::packetControl(const CommandPacketControl& packet)
{
    //Serial.printf("Control packet type:%d, len:%d, value:%d\r\n", packet.type, packet.len, packet.value);
    switch (packet.control) {
    case CommandPacketControl::MOTORS_SWITCH_OFF:
        _flightController.motorsSwitchOff();
        return true;
    case CommandPacketControl::MOTORS_SWITCH_ON:
        _flightController.motorsSwitchOn();
        return true;
    case CommandPacketControl::RESET:
        //!!_flightController.motorsResetEncodersToZero();
        return true;
    case CommandPacketControl::SET_MODE:
        //!!_flightController.setControlMode(static_cast<MotorPairController::control_mode_e>(packet.value));
        return true;
    case CommandPacketControl::SET_PID_PROFILE:
        return false; // not implemented
    case CommandPacketControl::SET_RATES_PROFILE:
        return false; // not implemented
    case CommandPacketControl::SET_DEBUG_MODE:
        _flightController.getDebug().setMode(static_cast<debug_mode_e>(packet.value));
        return true;
    default:
        // do nothing
        break;
    } // end switch
    return false;
}

bool BackchannelFlightController::packetSetPID(const CommandPacketSetPID& packet)
{
    //Serial.printf("SetPID packet type:%d, len:%d, pidIndex:%d setType:%d value:%3d f0:%f\r\n", packet.type, packet.len, packet.pidIndex, packet.setType, packet.value, packet.f0);

    const auto pidIndex = static_cast<FlightController::pid_index_e>(packet.pidIndex);
    if (pidIndex >= FlightController::PID_COUNT) {
        //Serial.printf("Backchannel::packetSetPID invalid pidIndex:%d\r\n", packet.pidIndex);
        return false;
    }

    bool transmit = false;
    switch (packet.setType) {
    case CommandPacketSetPID::SET_P:
        _flightController.setPID_P_MSP(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_I:
        _flightController.setPID_I_MSP(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_D:
        _flightController.setPID_D_MSP(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_F:
        _flightController.setPID_F_MSP(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_S:
        _flightController.setPID_S_MSP(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_PD: {
        // Set P and change D to preserve P/D ratio
        const PIDF pid = _flightController.getPID(pidIndex);
        const float ratio = pid.getD() / pid.getP();
        _flightController.setPID_P_MSP(pidIndex, packet.value);
        _flightController.setPID_D(pidIndex, _flightController.getPID(pidIndex).getP() * ratio);
        transmit = true;
        break;
    }
    case CommandPacketSetPID::SAVE_P: // NOLINT(bugprone-branch-clone) false positive
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_I:
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_D:
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_F:
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_S:
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_PD:
        //Serial.printf("Saved PID packetType:%d pidIndex:%d setType:%d\r\n", packet.type, packet.pidIndex, packet.setType);
        // Currently we don't save individual PID constants: if any save request is received we save all the PID constants.
        _nonVolatileStorage.storePID(_flightController.getPID_Constants(pidIndex), pidIndex);
        return true;
    case CommandPacketSetPID::RESET_PID:
        _nonVolatileStorage.resetPID(pidIndex);
        return true;
    default:
        //Serial.printf("Backchannel::packetSetPID invalid setType:%d\r\n", packet.pidIndex);
        break;
    }

    if (transmit) {
        // send back the new data for display
        const size_t len = packTelemetryData_PID(
            _transmitDataBufferPtr,
            _telemetryID,
            _sequenceNumber,
            _flightController,
            _flightController.getControlMode(),
            0.0F,
            0.0F
        );
        sendData(_transmitDataBufferPtr, len);
        return true;
    }
    return false;
}

bool BackchannelFlightController::sendPacket(uint8_t subCommand)
{
    if (_requestType == CommandPacketRequestData::REQUEST_AHRS_DATA) {
        // intercept an AHRS_DATA request to replace roll and pitch values
        const Quaternion orientationENU = _ahrs.getOrientationForInstrumentationUsingLock();

        const size_t len = packTelemetryData_AHRS(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _ahrs, _vehicleController);
        TD_AHRS* td = reinterpret_cast<TD_AHRS*>(_transmitDataBufferPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)
        // convert from ENU to NED
        td->data.roll = -orientationENU.calculatePitchDegrees(),
        td->data.pitch = orientationENU.calculateRollDegrees(),
        td->data.yaw = orientationENU.calculateYawDegrees(),

        sendData(_transmitDataBufferPtr, len);
        return true;
    }

    if (BackchannelStabilizedVehicle::sendPacket(subCommand)) {
        // if the base class has sent the packet then we have nothing to do
        return true;
    }

    switch (_requestType) {
    case CommandPacketRequestData::REQUEST_PID_DATA: {
        const size_t len = packTelemetryData_PID(
            _transmitDataBufferPtr,
            _telemetryID,
            _sequenceNumber,
            _flightController,
            _flightController.getControlMode(),
            0.0F,
            0.0F
        );
        //Serial.printf("pidLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        _requestType = CommandPacketRequestData::NO_REQUEST; // reset _requestType to NO_REQUEST, since REQUEST_PID_DATA is a one shot, as response to keypress
        break;
    }
    case CommandPacketRequestData::REQUEST_VEHICLE_CONTROLLER_DATA: {
        const size_t len = packTelemetryData_FC_QUADCOPTER(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _flightController);
        //Serial.printf("vcLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }
    /*case CommandPacketRequestData::REQUEST_DEBUG_DATA: {
        const size_t len = packTelemetryData_Debug(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _flightController.getDebug());
        //Serial.printf("debugLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }*/
    case CommandPacketRequestData::REQUEST_MSP_DATA: {
        (void)subCommand;
        //const size_t len = packTelemetryData_MSP(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _msp, subCommand);
        //if (len <= ESP_NOW_MAX_DATA_LEN) {
        //    sendData(_transmitDataBufferPtr, len);
        //}
        break;
    }
    default:
        return false;
    } // end switch
    return true;
}
