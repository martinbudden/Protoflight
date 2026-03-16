#pragma once

#include <blackbox_callbacks_base.h>

class AhrsMessageQueue;
class Cockpit;
class Debug;
class FlightController;
class GPS;
class ImuFilters;
class MotorMixerBase;
class RcModes;
class ReceiverBase;

struct blackbox_context_t {
    const AhrsMessageQueue& ahrs_message_queue;
    const FlightController& flight_controller;
    const ImuFilters& imu_filters;
    const MotorMixerBase& motor_mixer;
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
    virtual void load_slow_state(blackbox_slow_state_t& slow_state, const blackbox_context_t& ctx) override;
    virtual void load_main_state(blackbox_main_state_t& main_state, uint32_t current_time_us, const blackbox_context_t& ctx) override;
    virtual void load_gps_state(blackbox_gps_state_t& gps_state, const blackbox_context_t& ctx) override;

    virtual bool is_armed(const blackbox_context_t& ctx) const override;
    virtual bool is_blackbox_mode_active(const blackbox_context_t& ctx) const override;
    virtual bool is_blackbox_erase_mode_active(const blackbox_context_t& ctx) const override;
    virtual bool is_blackbox_mode_activation_condition_present(const blackbox_context_t& ctx) const override;
    virtual uint32_t get_arming_beep_time_microseconds(const blackbox_context_t& ctx) const override;
    virtual bool are_motors_running(const blackbox_context_t& ctx) const override;
    virtual uint32_t rc_mode_activation_mask(const blackbox_context_t& ctx) const override;
    virtual void beep(const blackbox_context_t& ctx) const override;
};
