#include "Cockpit.h"
#include "FlightController.h"
#include "MSP_Protoflight.h"

#include <MSP_Protocol.h>


MSP_Protoflight::MSP_Protoflight(AHRS& ahrs, FlightController& flightController, Cockpit& cockpit, const ReceiverBase& receiver, const Autopilot& autopilot, const IMU_Filters& imuFilters, Debug& debug, NonVolatileStorage& nvs, Blackbox* blackbox, VTX_Base* vtx, OSD* osd) :
    _ahrs(ahrs),
    _flightController(flightController),
    _cockpit(cockpit),
    _receiver(receiver),
    _autopilot(autopilot),
    _imuFilters(imuFilters),
    _debug(debug),
    _nonVolatileStorage(nvs),
    _blackbox(blackbox),
    _vtx(vtx),
    _osd(osd)
{
    //_mspBox.init(features, ahrs, flightController);
    enum { MSP_OVERRIDE_OFF = false, AIRMODE_OFF = false, ANTI_GRAVITY_OFF = false };
    enum { ACCELEROMETER_AVAILABLE = true };
    _mspBox.init(
        ACCELEROMETER_AVAILABLE,
        _cockpit.featureIsEnabled(Features::FEATURE_INFLIGHT_ACC_CAL),
        MSP_OVERRIDE_OFF,
        AIRMODE_OFF,
        ANTI_GRAVITY_OFF
    );
}

void MSP_Protoflight::rebootFn(serialPort_t* serialPort)
{
    (void)serialPort;

    _flightController.motorsSwitchOff();

    switch (_rebootMode) {
    case REBOOT_FIRMWARE:
        //systemReset();
        break;
    case REBOOT_BOOTLOADER_ROM:
        //systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
        break;
    case REBOOT_MSC:
        [[fallthrough]];
    case REBOOT_MSC_UTC: {
        //const int16_t timezoneOffsetMinutes = 0;//(_rebootMode == REBOOT_MSC) ? timeConfig()->tz_offsetMinutes : 0;
        //systemResetToMsc(timezoneOffsetMinutes);
        break;
    }
    case REBOOT_BOOTLOADER_FLASH:
        //systemResetToBootloader(BOOTLOADER_REQUEST_FLASH);
        break;
    default:
        return;
    }
}

MSP_Base::result_e MSP_Protoflight::processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn, StreamBufReader& src)
{
    switch (cmdMSP) {
    case MSP_BOXNAMES: {
        const size_t page = src.bytesRemaining() ? src.readU8() : 0;
        _mspBox.serializeBoxReplyBoxName(dst, page);
        break;
    }
    case MSP_BOXIDS: {
        const size_t page = src.bytesRemaining() ? src.readU8() : 0;
        _mspBox.serializeBoxReplyPermanentId(dst, page);
        break;
    }
    case MSP_REBOOT:
        if (src.bytesRemaining()) {
            _rebootMode = src.readU8();
            if (_rebootMode >= REBOOT_COUNT || _rebootMode == REBOOT_MSC || _rebootMode == REBOOT_MSC_UTC) {
                return RESULT_ERROR;
            }
        } else {
            _rebootMode = REBOOT_FIRMWARE;
        }

        dst.writeU8(_rebootMode);

        if (postProcessFn) {
                *postProcessFn = static_cast<MSP_Base::postProcessFnPtr>(&MSP_Protoflight::rebootFn);
        }

        break;
    case MSP_RESET_CONF: {
        if (src.bytesRemaining() >= 1) {
            // Added in MSP API 1.42
            src.readU8();
        }

        const bool success = false;
        if (!_cockpit.isArmed()) {
            //success = resetEEPROM(); //!!TODO: implement this
            //if (success && postProcessFn) {
            if (postProcessFn) {
                _rebootMode = REBOOT_FIRMWARE;
                *postProcessFn = static_cast<MSP_Base::postProcessFnPtr>(&MSP_Protoflight::rebootFn);
            }
        }
        // Added in API version 1.42
        dst.writeU8(success);
        break;
    }
    default:
        return processOutCommand(cmdMSP, dst, srcDesc, postProcessFn);
    }
    return RESULT_ACK;
}
