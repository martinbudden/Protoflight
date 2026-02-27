# pragma once

#include <cstddef>
#include <cstdint>

class FlightController;
class MotorMixerBase;
class MspBase;
class Debug;
struct msp_parameter_group_t;


size_t pack_telemetry_data_fc_quadcopter(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, const FlightController& flightController, const MotorMixerBase& motorMixer); // NOLINT(readability-avoid-const-params-in-decls) false positive

size_t pack_telemetry_data_debug(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, const Debug& debug); // NOLINT(readability-avoid-const-params-in-decls)

size_t pack_telemetry_data_msp(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, msp_parameter_group_t& pg, MspBase& msp, int16_t cmd_msp);
