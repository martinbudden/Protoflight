#pragma once

#include "MSP_ProtoFlightBox.h"
#include <MSP_Base.h>


class AHRS;
class Autopilot;
class Debug;
class Features;
class FlightController;
class NonVolatileStorage;
class RadioController;
class ReceiverBase;


class MSP_ProtoFlight : public MSP_Base {
public:
    enum { RATEPROFILE_MASK = (1 << 7) };
    enum { RTC_NOT_SUPPORTED = 0xFF };
    enum { SENSOR_NOT_AVAILABLE = 0xFF };
public:
    virtual ~MSP_ProtoFlight() = default;
    MSP_ProtoFlight(AHRS& ahrs, FlightController& flightController, RadioController& radioController, const ReceiverBase& receiver, const Autopilot& autopilot, Debug& debug, NonVolatileStorage& nvs, Features& features);

    virtual void rebootFn(serialPort_t* serialPort) override;

    virtual result_e processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) override;
    virtual result_e processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn, StreamBuf& src) override;
    result_e processOutCommand(int16_t cmdMSP, StreamBuf& dst) { return processOutCommand(cmdMSP, dst, 0, nullptr); };

    virtual result_e processInCommand(int16_t cmdMSP, StreamBuf& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) override;
    result_e processInCommand(int16_t cmdMSP, StreamBuf& src) { return processInCommand(cmdMSP, src, 0, nullptr); }
private:
    MSP_ProtoFlightBox _mspBox;
    AHRS& _ahrs;
    FlightController& _flightController;
    RadioController& _radioController;
    const ReceiverBase& _receiver;
    const Autopilot& _autopilot;
    Debug& _debug;
    NonVolatileStorage& _nonVolatileStorage;
    Features& _features;
    uint8_t _pidProfileIndex {0};
    uint8_t _ratesProfileIndex {0};
};