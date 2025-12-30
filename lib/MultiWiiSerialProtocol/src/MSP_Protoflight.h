#pragma once

#include "MSP_ProtoflightBox.h"
#include <MSP_Base.h>


class AHRS;
class Autopilot;
class Blackbox;
class Cockpit;
class Debug;
class FlightController;
class IMU_Filters;
class NonVolatileStorage;
class OSD;
class ReceiverBase;
class VTX;


class MSP_Protoflight : public MSP_Base {
public:
    virtual ~MSP_Protoflight() = default;
    MSP_Protoflight(AHRS& ahrs, FlightController& flightController, Cockpit& cockpit, const ReceiverBase& receiver, const IMU_Filters& imuFilters, Debug& debug, NonVolatileStorage& nvs, Blackbox* blackbox, VTX* vtx, OSD* osd);
public:
    enum { RATEPROFILE_MASK = (1 << 7) };
    enum { RTC_NOT_SUPPORTED = 0xFF };
    enum { SENSOR_NOT_AVAILABLE = 0xFF };
public:
    virtual void rebootFn(serialPort_t* serialPort) override;

    virtual result_e processGetCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) override;
    virtual result_e processGetCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn, StreamBufReader& src) override;
    result_e processGetCommand(int16_t cmdMSP, StreamBuf& dst) { return processGetCommand(cmdMSP, dst, 0, nullptr); };

    virtual result_e processSetCommand(int16_t cmdMSP, StreamBufReader& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) override;
    result_e processSetCommand(int16_t cmdMSP, StreamBufReader& src) { return processSetCommand(cmdMSP, src, 0, nullptr); }
private:
    void serializeVTX(StreamBuf& dst);
private:
    MSP_ProtoflightBox _mspBox;
    AHRS& _ahrs;
    FlightController& _flightController;
    Cockpit& _cockpit;
    const ReceiverBase& _receiver;
    const Autopilot& _autopilot;
    const IMU_Filters& _imuFilters;
    Debug& _debug;
    NonVolatileStorage& _nonVolatileStorage;
    Blackbox* _blackbox;
    VTX* _vtx;
    OSD* _osd;
    uint8_t _pidProfileIndex {0};
    uint8_t _ratesProfileIndex {0};
};