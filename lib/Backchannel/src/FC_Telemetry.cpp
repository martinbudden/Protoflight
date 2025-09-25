#include "FC_Telemetry.h"

#include "FC_TelemetryData.h"

#include <Debug.h>
#include <MSP_Base.h>
#include <SV_TelemetryData.h>
#include <StreamBuf.h>

#include <cstring>


#if false
/*!
Packs the FlightController PID telemetry data into a TD_FC_PIDS packet. Returns the length of the packet.
*/
size_t packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const FlightController& flightController)
{
    static_assert(static_cast<int>(TD_FC_PIDS::PID_COUNT) == static_cast<int>(FlightController::PID_COUNT));
    TD_FC_PIDS* td = reinterpret_cast<TD_FC_PIDS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_FC_PIDS::TYPE;
    td->len = sizeof(TD_FC_PIDS);
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    for (size_t ii = FlightController::PID_BEGIN; ii < FlightController::PID_COUNT; ++ii) {
        td->spids[ii].setpoint = flightController.getPID_Setpoint(static_cast<FlightController::pid_index_e>(ii));
        td->spids[ii].pid = flightController.getPID_Constants(static_cast<FlightController::pid_index_e>(ii));
    }

    return td->len;
}
#endif

/*!
Packs the FlightController telemetry data into a TD_FC_QUADCOPTER packet. Returns the length of the packet.
*/
size_t packTelemetryData_FC_QUADCOPTER(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const FlightController& flightController)
{
    TD_FC_QUADCOPTER* td = reinterpret_cast<TD_FC_QUADCOPTER*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_FC_QUADCOPTER::TYPE;
    td->len = sizeof(TD_FC_QUADCOPTER);
    //td->subType = (TD_FC_QUADCOPTER::PID_COUNT << 4U) | TD_FC_QUADCOPTER::MOTOR_COUNT;
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    //!!td->taskIntervalTicks = static_cast<uint8_t>(vehicleControllerTask.getTickCountDelta());

    td->flags = flightController.motorsIsOn() ? TD_FC_QUADCOPTER::MOTORS_ON_FLAG : 0x00;
    td->flags |= static_cast<uint8_t>(TD_FC_QUADCOPTER::CONTROL_MODE_MASK & flightController.getControlMode()); // NOLINT(hicpp-signed-bitwise)

    const flight_controller_quadcopter_telemetry_t telemetryData = flightController.getTelemetryData();
    //!!memcpy(&td->data, &telemetryData, sizeof(flight_controller_quadcopter_telemetry_t));
    //!!td->pidErrors[TD_FC_QUADCOPTER::ROLL_RATE_DPS] = telemetryData.rollRateError;
    for (size_t ii = 0; ii < TD_FC_QUADCOPTER::MOTOR_COUNT; ++ii) {
        td->data.motors[ii].power = telemetryData.motors[ii].power;
        td->data.motors[ii].rpm = telemetryData.motors[ii].rpm;
    }

    return td->len;
};

size_t packTelemetryData_MSP(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, MSP_Base& msp, int16_t cmdMSP)
{
    (void) sequenceNumber;

    TD_MSP* td = reinterpret_cast<TD_MSP*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)
    td->id = id;

    td->data.msp.headerDollar = '$';
    td->data.msp.headerM = 'M';
    td->data.msp.headerDirection = '>';
    td->data.msp.payloadSize = 0;
    td->data.msp.messageType = static_cast<uint8_t>(cmdMSP);

    StreamBuf sbuf(&td->data.msp.payload[0], sizeof(TD_MSP));

    msp.processOutCommand(cmdMSP, sbuf, 0, nullptr);
    const auto payloadSize = static_cast<uint8_t>(sbuf.bytesWritten());
    uint8_t checksum = 0;
    for (size_t ii = 0; ii < payloadSize; ++ii) {
        checksum ^= td->data.msp.payload[ii];
    }
    td->data.msp.payloadSize = payloadSize;
    td->data.msp.payload[payloadSize] = checksum;

    return payloadSize + TD_MSP::PACKET_OVERHEAD;
};

/*!
Packs the TD_Debug packet. Returns the length of the packet.
*/
size_t packTelemetryData_Debug(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const Debug& debug)
{
    TD_DEBUG* td = reinterpret_cast<TD_DEBUG*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TASK_INTERVALS::TYPE;
    td->len = sizeof(TD_TASK_INTERVALS);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    static_assert(static_cast<int>(TD_DEBUG::VALUE_COUNT) == static_cast<int>(Debug::VALUE_COUNT));
    for (size_t ii = 0; ii < TD_DEBUG::VALUE_COUNT; ++ii) {
        td->values[ii] = debug.get(ii);
    }
    //!!TODO: add mode to TD_Debug packet
    //td->mode = debug.getMode();

    return td->len;
}
