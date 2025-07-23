#pragma once

#include "MSP_Base.h"
#include "MSP_ProtoBox.h"


class AHRS;
class Features;
class FlightController;
class RadioController;
class ReceiverBase;


class MSP_ProtoFlight : public MSP_Base {
public:
    enum { RATEPROFILE_MASK = (1 << 7) };
    enum { RTC_NOT_SUPPORTED = 0xFF };
    enum { SENSOR_NOT_AVAILABLE = 0xFF };
public:
    virtual ~MSP_ProtoFlight() = default;
    MSP_ProtoFlight(Features& features, AHRS& ahrs, FlightController& flightController, RadioController& radioController, ReceiverBase& receiver);

    virtual void rebootFn(serialPort_t* serialPort) override;

    virtual result_e processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) override;
    virtual result_e processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn, StreamBuf& src) override;
    result_e processOutCommand(int16_t cmdMSP, StreamBuf& dst) { return processOutCommand(cmdMSP, dst, 0, nullptr); };

    virtual result_e processInCommand(int16_t cmdMSP, StreamBuf& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) override;
    result_e processInCommand(int16_t cmdMSP, StreamBuf& src) { return processInCommand(cmdMSP, src, 0, nullptr); }
private:
    MSP_ProtoBox _mspBox;
    Features& _features;
    AHRS& _ahrs;
    FlightController& _flightController;
    RadioController& _radioController;
    ReceiverBase& _receiver;
};