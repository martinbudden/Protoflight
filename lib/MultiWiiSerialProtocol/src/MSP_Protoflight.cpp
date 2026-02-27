#include "Cockpit.h"
#include "FlightController.h"
#include "MSP_Protoflight.h"

#include <MSP_Protocol.h>



void MSP_Protoflight::reboot_fn(msp_parameter_group_t& pg, serialPort_t* serialPort) // NOLINT(readability-make-member-function-const)
{
    (void)pg;
    (void)serialPort;

    pg.flightController.motorsSwitchOff(pg.motorMixer);

    switch (_reboot_mode) {
    case REBOOT_FIRMWARE:
        //systemReset();
        break;
    case REBOOT_BOOTLOADER_ROM:
        //systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
        break;
    case REBOOT_MSC:
        [[fallthrough]];
    case REBOOT_MSC_UTC: {
        //const int16_t timezoneOffsetMinutes = 0;//(_reboot_mode == REBOOT_MSC) ? timeConfig()->tz_offsetMinutes : 0;
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

msp_result_e MSP_Protoflight::set_passthrough_command(msp_parameter_group_t& pg, StreamBufWriter& dst, StreamBufReader& src) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)pg;

    const size_t dataSize = src.bytes_remaining();
    if (dataSize == 0) {
        // Legacy format
        _passthrough_mode = PASSTHROUGH_ESC_4WAY;
    } else {
        _passthrough_mode = src.read_u8();
        _passthrough_argument = src.read_u8();
    }

    switch (_passthrough_mode) {
    case PASSTHROUGH_SERIAL_ID:
    case PASSTHROUGH_SERIAL_FUNCTION_ID:
#if false
        if (findPassthroughSerialPort()) {
            if (postProcessFn) {
                *postProcessFn = serialPassthroughFn;
            }
            dst.write_u8(1);
        } else {
            dst.write_u8(0);
        }
#endif
        break;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    case MSP_PASSTHROUGH_ESC_4WAY:
        // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
        dst.write_u8(esc4wayInit());

        if (mspPostProcessFn) {
            *mspPostProcessFn = esc4wayProcess;
        }
        break;

#ifdef USE_ESCSERIAL
    case MSP_PASSTHROUGH_ESC_SIMONK:
    case MSP_PASSTHROUGH_ESC_BLHELI:
    case MSP_PASSTHROUGH_ESC_KISS:
    case MSP_PASSTHROUGH_ESC_KISSALL:
    case MSP_PASSTHROUGH_ESC_CASTLE:
        if (mspPassthroughArgument < getMotorCount() || (mspPassthroughMode == MSP_PASSTHROUGH_ESC_KISS && mspPassthroughArgument == ALL_MOTORS)) {
            dst.write_u8(1);

            if (mspPostProcessFn) {
                *mspPostProcessFn = mspEscPassthroughFn;
            }

            break;
        }
        FALLTHROUGH;
#endif // USE_ESCSERIAL
#endif // USE_SERIAL_4WAY_BLHELI_INTERFACE
    default:
        dst.write_u8(0);
    }
    return MSP_RESULT_ACK;
}

msp_result_e MSP_Protoflight::process_get_set_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufWriter& dst, StreamBufReader& src)
{
    switch (cmd_msp) {
    case MSP_BOXNAMES: {
        const size_t page = (src.bytes_remaining() > 0) ? src.read_u8() : 0;
        pg.cockpit.serialize_box_reply_box_name(dst, page);
        break;
    }
    case MSP_BOXIDS: {
        const size_t page = (src.bytes_remaining() > 0) ? src.read_u8() : 0;
        pg.cockpit.serialize_box_reply_permanent_id(dst, page);
        break;
    }
    case MSP_SET_PASSTHROUGH:
    case MSP_REBOOT:
        if (src.bytes_remaining() > 0) {
            _reboot_mode = src.read_u8();
            if (_reboot_mode >= REBOOT_COUNT || _reboot_mode == REBOOT_MSC || _reboot_mode == REBOOT_MSC_UTC) {
                return MSP_RESULT_ERROR;
            }
        } else {
            _reboot_mode = REBOOT_FIRMWARE;
        }

        dst.write_u8(_reboot_mode);

        break;
#ifdef USE_FLASHFS
    case MSP_DATAFLASH_READ: {
            return mspFcDataFlashReadCommand(pg, dst, src);
#endif
    case MSP_RESET_CONF: {
        if (src.bytes_remaining() >= 1) {
            // Added in MSP API 1.42
            src.read_u8();
        }

        const bool success = false;
        if (!pg.cockpit.isArmed()) {
            //success = resetEEPROM(); //!!TODO: implement this
            //if (success && postProcessFn) {
                _reboot_mode = REBOOT_FIRMWARE;
            //}
        }
        // Added in API version 1.42
        dst.write_u8(success);
        break;
    }
    default:
        return process_get_command(pg, cmd_msp, dst);
    }
    return MSP_RESULT_ACK;
}
