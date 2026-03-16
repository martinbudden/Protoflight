#pragma once

#include <msp_base.h>


class Ahrs;
class AhrsMessageQueue;
class Autopilot;
class Blackbox;
class Cockpit;
class Debug;
class FlightController;
class GPS;
class ImuFilters;
class MotorMixerBase;
class NonVolatileStorage;
class OSD;
class RcModes;
class ReceiverBase;
class VTX;

struct serial_port_t;

struct msp_context_t {
    Ahrs& ahrs;
    FlightController& flight_controller;
    const AhrsMessageQueue& ahrs_message_queue;
    MotorMixerBase& motor_mixer;
    Cockpit& cockpit;
    const ReceiverBase& receiver;
    RcModes& rc_modes;
    ImuFilters& imu_filters;
    Debug& debug;
    NonVolatileStorage& nvs;
    Blackbox* blackbox;
    VTX* vtx;
    OSD* osd;
    GPS* gps;
};

class MSP_Protoflight : public MspBase {
public:
    virtual ~MSP_Protoflight() = default;
    MSP_Protoflight() = default;
public:
    static constexpr uint8_t RATEPROFILE_MASK = (1 << 7);
    static constexpr uint8_t RTC_NOT_SUPPORTED = 0xFF;
    static constexpr uint8_t SENSOR_NOT_AVAILABLE = 0xFF;
public:
    void reboot_fn(msp_context_t& ctx, serial_port_t* serial_port);

    virtual msp_result_e process_write_command(msp_context_t& ctx, int16_t cmd_msp, StreamBufWriter& dst, StreamBufReader& src) override;
    msp_result_e process_get_command(msp_context_t& ctx, int16_t cmd_msp, StreamBufWriter& dst);

    virtual msp_result_e process_read_command(msp_context_t& ctx, int16_t cmd_msp, StreamBufReader& src) override;
    msp_result_e set_passthrough_command(msp_context_t& ctx, StreamBufWriter& dst, StreamBufReader& src);
private:
    void serialize_vtx(msp_context_t& ctx, StreamBufWriter& dst);
    void serialize_dataflash_summary_reply(msp_context_t& ctx, StreamBufWriter& dst);
    void serialize_sd_card_summary_reply(msp_context_t& ctx, StreamBufWriter& dst);
    void set_msp_vtx_device_status_ready(msp_context_t& ctx);
protected:
    uint8_t _passthrough_mode {};
    uint8_t _passthrough_argument {};
    uint8_t _reboot_mode {};
};
