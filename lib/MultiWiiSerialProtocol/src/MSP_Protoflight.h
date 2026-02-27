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
class IMU_Filters;
class MotorMixerBase;
class NonVolatileStorage;
class OSD;
class RcModes;
class ReceiverBase;
class VTX;

struct msp_parameter_group_t {
    Ahrs& ahrs;
    FlightController& flightController;
    const AhrsMessageQueue& ahrsMessageQueue;
    MotorMixerBase& motorMixer;
    Cockpit& cockpit;
    const ReceiverBase& receiver;
    RcModes& rc_modes;
    IMU_Filters& imuFilters;
    Debug& debug;
    NonVolatileStorage& nonVolatileStorage;
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
    enum { RATEPROFILE_MASK = (1 << 7) };
    enum { RTC_NOT_SUPPORTED = 0xFF };
    enum { SENSOR_NOT_AVAILABLE = 0xFF };
public:
    void reboot_fn(msp_parameter_group_t& pg, serialPort_t* serialPort);

    virtual msp_result_e process_get_set_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufWriter& dst, StreamBufReader& src) override;
    msp_result_e process_get_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufWriter& dst);

    virtual msp_result_e process_set_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufReader& src) override;
    msp_result_e set_passthrough_command(msp_parameter_group_t& pg, StreamBufWriter& dst, StreamBufReader& src);
private:
    void serializeVTX(msp_parameter_group_t& pg, StreamBufWriter& dst);
    void serializeDataflashSummaryReply(msp_parameter_group_t& pg, StreamBufWriter& dst);
    void serializeSDCardSummaryReply(msp_parameter_group_t& pg, StreamBufWriter& dst);
    void setMSP_VTX_DeviceStatusReady(msp_parameter_group_t& pg);
protected:
    uint8_t _passthrough_mode {};
    uint8_t _passthrough_argument {};
    uint8_t _reboot_mode {};
};
