# pragma once

#include "FlightController.h"
#include <VehicleControllerTask.h>

#include <cstddef>
#include <cstdint>

class MSP_Base;
class Debug;

size_t packTelemetryData_FC_QUADCOPTER(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const FlightController& flightController); // NOLINT(readability-avoid-const-params-in-decls) false positive

size_t packTelemetryData_Debug(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const Debug& debug); // NOLINT(readability-avoid-const-params-in-decls)

size_t packTelemetryData_MSP(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, MSP_Base& msp, int16_t cmdMSP);
