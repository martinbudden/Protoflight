#include "Cockpit.h"
#include "FlightController.h"
#include "MSP_Protoflight.h"

#include <MSP_Protocol.h>


MSP_Protoflight::MSP_Protoflight(AHRS& ahrs, FlightController& flightController, Cockpit& cockpit, const ReceiverBase& receiver, const IMU_Filters& imuFilters, Debug& debug, NonVolatileStorage& nvs, Blackbox* blackbox, VTX* vtx, OSD* osd, GPS* gps) :
    _ahrs(ahrs),
    _flightController(flightController),
    _cockpit(cockpit),
    _receiver(receiver),
    _imuFilters(imuFilters),
    _debug(debug),
    _nonVolatileStorage(nvs),
    _blackbox(blackbox),
    _vtx(vtx),
    _osd(osd),
    _gps(gps)
{
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

MSP_Base::result_e MSP_Protoflight::processGetCommand(int16_t cmdMSP, StreamBufWriter& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn, StreamBufReader& src)
{
    switch (cmdMSP) {
    case MSP_BOXNAMES: {
        const size_t page = (src.bytes_remaining() > 0) ? src.read_u8() : 0;
        _cockpit.serialize_box_reply_box_name(dst, page);
        break;
    }
    case MSP_BOXIDS: {
        const size_t page = (src.bytes_remaining() > 0) ? src.read_u8() : 0;
        _cockpit.serialize_box_reply_permanent_id(dst, page);
        break;
    }
    case MSP_REBOOT:
        if (src.bytes_remaining() > 0) {
            _rebootMode = src.read_u8();
            if (_rebootMode >= REBOOT_COUNT || _rebootMode == REBOOT_MSC || _rebootMode == REBOOT_MSC_UTC) {
                return RESULT_ERROR;
            }
        } else {
            _rebootMode = REBOOT_FIRMWARE;
        }

        dst.write_u8(_rebootMode);

        if (postProcessFn) {
            *postProcessFn = static_cast<MSP_Base::postProcessFnPtr>(&MSP_Protoflight::rebootFn);
        }

        break;
    case MSP_RESET_CONF: {
        if (src.bytes_remaining() >= 1) {
            // Added in MSP API 1.42
            src.read_u8();
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
        dst.write_u8(success);
        break;
    }
    default:
        return processGetCommand(cmdMSP, dst, srcDesc, postProcessFn);
    }
    return RESULT_ACK;
}
