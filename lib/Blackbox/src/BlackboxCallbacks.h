#pragma once

#include <blackbox_callbacks_base.h>

class AhrsMessageQueue;
class Cockpit;
class Debug;
class FlightController;
class GPS;
class IMU_Filters;
class MotorMixerBase;
class RcModes;
class ReceiverBase;

struct blackbox_parameter_group_t {
    const AhrsMessageQueue& ahrs_message_queue;
    const FlightController& flightController;
    const IMU_Filters& imuFilters;
    const MotorMixerBase& motorMixer;
    const Cockpit& cockpit;
    const ReceiverBase& receiver;
    const RcModes& rc_modes;
    const Debug& debug;
    const GPS* gps;
};


class BlackboxCallbacks : public BlackboxCallbacksBase {
public:
    BlackboxCallbacks() = default;
public:
    virtual void load_slow_state(blackbox_slow_state_t& blackboxSlowState, const blackbox_parameter_group_t& pg) override;
    virtual void load_main_state(blackbox_main_state_t& blackboxMainState, uint32_t currentTimeUs, const blackbox_parameter_group_t& pg) override;
    virtual void load_gps_state(blackbox_gps_state_t& gpsState, const blackbox_parameter_group_t& pg) override;

    virtual bool is_armed(const blackbox_parameter_group_t& pg) const override;
    virtual bool is_blackbox_mode_active(const blackbox_parameter_group_t& pg) const override;
    virtual bool is_blackbox_erase_mode_active(const blackbox_parameter_group_t& pg) const override;
    virtual bool is_blackbox_mode_activation_condition_present(const blackbox_parameter_group_t& pg) const override;
    virtual uint32_t get_arming_beep_time_microseconds(const blackbox_parameter_group_t& pg) const override;
    virtual bool are_motors_running(const blackbox_parameter_group_t& pg) const override;
    virtual uint32_t rc_mode_activation_mask(const blackbox_parameter_group_t& pg) const override;
    virtual void beep(const blackbox_parameter_group_t& pg) const override;
};
