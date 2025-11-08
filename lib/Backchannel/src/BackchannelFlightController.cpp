#include "BackchannelFlightController.h"

#include "FC_Telemetry.h"

#include <AHRS.h>
#if !defined(FRAMEWORK_TEST)
#include <AHRS_MessageQueue.h>
#endif
#include <BackchannelTransceiverBase.h>
#include <Debug.h>
//#define USE_DEBUG_PRINTF_BACKCHANNEL
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
#include <HardwareSerial.h>
#endif
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
        const TaskBase* dashboardTask,
        NonVolatileStorage& nvs
    ) :
    BackchannelStabilizedVehicle(
        backchannelTransceiver,
        backchannelMacAddress,
        myMacAddress,
        flightController,
        ahrs,
        receiver,
        dashboardTask
    ),
    _flightController(flightController),
    _nonVolatileStorage(nvs)
{
#if !defined(ESP_NOW_MAX_DATA_LEN)
#define ESP_NOW_MAX_DATA_LEN (250)
#endif
    static_assert(sizeof(TD_MPC) <= ESP_NOW_MAX_DATA_LEN); // 124
    static_assert(sizeof(TD_SBR_PID) <= ESP_NOW_MAX_DATA_LEN); // 96
}

bool BackchannelFlightController::packetSetOffset(const CommandPacketSetOffset& packet)
{
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
    //Serial.printf("    packetSetOffset itype:%d, len:%d value:%d, type:%d\r\n", packet.type, packet.len, packet.value, packet.type);
#endif
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

    static_assert(static_cast<int>(TD_PID::MAX_PID_COUNT) >= static_cast<int>(FlightController::PID_COUNT));

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
    case CommandPacketSetPID::SET_S:
        _flightController.setPID_S_MSP(pidIndex, packet.value);
        transmit = true;
        break;
    case CommandPacketSetPID::SET_K:
        _flightController.setPID_K_MSP(pidIndex, packet.value);
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
    case CommandPacketSetPID::SAVE_S:
        [[fallthrough]];
    case CommandPacketSetPID::SAVE_K:
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
#if !defined(FRAMEWORK_TEST)
    if (_requestType == CommandPacketRequestData::REQUEST_AHRS_DATA) {
        // intercept AHRS_DATA request to replace roll and pitch values
        AHRS::ahrs_data_t ahrsData;
        _flightController.getAHRS_MessageQueue().PEEK_AHRS_DATA(ahrsData);
        const size_t len = packTelemetryData_AHRS(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _ahrs, ahrsData);
        TD_AHRS* td = reinterpret_cast<TD_AHRS*>(_transmitDataBufferPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)
        // convert from ENU to NED
        const Quaternion orientationENU = ahrsData.orientation;
        td->data.roll = -orientationENU.calculatePitchDegrees(),
        td->data.pitch = orientationENU.calculateRollDegrees(),
        td->data.yaw = orientationENU.calculateYawDegrees(),
        sendData(_transmitDataBufferPtr, len);
        return true;
    }
#if defined(USE_FLIGHT_CONTROLLER_TIME_CHECKS)
    if (_requestType == CommandPacketRequestData::REQUEST_TASK_INTERVAL_EXTENDED_DATA) {
        // intercept AHRS_DATA request add additional timing information
        const size_t len = packTelemetryData_TaskIntervalsExtended(_transmitDataBufferPtr, _telemetryID, _sequenceNumber,
            _ahrs,
            _vehicleController,
            _mainTask ? _mainTask->getTickCountDelta() : 0,
            _backchannelTransceiver.getTickCountDeltaAndReset(),
            static_cast<uint32_t>(_receiver.getDroppedPacketCountDelta())
        );
        // use empty slots to add additional time checks
        // timings [0-3] are set in AHRS readIMUandUpdateOrientation(), timings [4-7] are free to use
        TD_TASK_INTERVALS_EXTENDED* td = reinterpret_cast<TD_TASK_INTERVALS_EXTENDED*>(_transmitDataBufferPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)
        td->data.ahrsTimeChecksMicroseconds[4] = _flightController.getTimeChecksMicroseconds(0);
        td->data.ahrsTimeChecksMicroseconds[5] = _flightController.getTimeChecksMicroseconds(1);
        td->data.ahrsTimeChecksMicroseconds[6] = _flightController.getTimeChecksMicroseconds(2);
        td->data.ahrsTimeChecksMicroseconds[7] = _flightController.getTimeChecksMicroseconds(3);
        //Serial.printf("tiLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        return true;
    }
#endif
#endif

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
    case CommandPacketRequestData::REQUEST_DEBUG_DATA: {
        const size_t len = packTelemetryData_Debug(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _flightController.getDebug());
        //Serial.printf("debugLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }
#if false
    case CommandPacketRequestData::REQUEST_MSP_DATA: {
        const size_t len = packTelemetryData_MSP(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _msp, subCommand);
        if (len <= ESP_NOW_MAX_DATA_LEN) {
            sendData(_transmitDataBufferPtr, len);
        }
        break;
    }
#endif
    default:
        return false;
    } // end switch
    return true;
}
