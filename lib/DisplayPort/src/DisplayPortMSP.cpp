#include "DisplayPortMSP.h"
#include <array>
#include <cassert>
#include <cstring>


DisplayPortMSP::DisplayPortMSP(MSP_SerialBase& mspSerial) :
    _mspSerial(mspSerial)
{
}

uint32_t DisplayPortMSP::output(uint8_t cmd, const uint8_t* buf, uint8_t len)
{
    (void)cmd;
    (void)buf;
    (void)len;
    //return mspSerialPush(displayPortSerial, cmd, buf, len, MSP_DIRECTION_REPLY, MSP_V1);
    return 0;
}

uint32_t DisplayPortMSP::clearScreen(display_clear_option_e options)
{
    (void)options;

    std::array<uint8_t, 1> cmd { COMMAND_CLEAR_SCREEN };

    return output(MSP_DISPLAYPORT, &cmd[0], sizeof(cmd));
}

bool DisplayPortMSP::drawScreen()
{
    std::array<uint8_t, 1> cmd { COMMAND_DRAW_SCREEN };
    output(MSP_DISPLAYPORT, &cmd[0], sizeof(cmd));

    return false;
}

uint32_t DisplayPortMSP::writeString(uint8_t x, uint8_t y, const char *text, uint8_t attr)
{
    enum { MSP_OSD_MAX_STRING_LENGTH = 30 };
    std::array<uint8_t, MSP_OSD_MAX_STRING_LENGTH + 4> buf;

    buf[0] = COMMAND_WRITE_STRING;
    buf[1] = x;
    buf[2] = y;
    buf[3] = 0;//!!displayPortProfileMsp()->fontSelection[attr & (SEVERITY_COUNT - 1)] & ATTR_FONT;

    if (attr & BLINK) {
        buf[3] |= ATTR_BLINK;
    }

    const auto len = static_cast<uint8_t>(strnlen(text, MSP_OSD_MAX_STRING_LENGTH));
    memcpy(&buf[4], text, len);

    return output(MSP_DISPLAYPORT, &buf[0], len + 4);
}

uint32_t DisplayPortMSP::writeChar(uint8_t x, uint8_t y, uint8_t c, uint8_t attr)
{
    std::array<char, 2> buf { static_cast<char>(c), 0 };

    return writeString(x, y, &buf[0], attr);
}

int DisplayPortMSP::heartbeat()
{
    std::array<uint8_t, 1> cmd { COMMAND_HEARTBEAT };

    // heartbeat is used to:
    // a) ensure display is not released by MW OSD software
    // b) prevent OSD Slave boards from displaying a 'disconnected' status.
    output(MSP_DISPLAYPORT, &cmd[0], sizeof(cmd));

    return 0;
}

int DisplayPortMSP::grab()
{
    clearScreen(DISPLAY_CLEAR_WAIT);
    ++_grabCount;
    return heartbeat();
}

uint32_t DisplayPortMSP::release()
{
    --_grabCount;
    std::array<uint8_t, 1> cmd { COMMAND_RELEASE };

    return output(MSP_DISPLAYPORT, &cmd[0], sizeof(cmd));
}

uint32_t DisplayPortMSP::writeSys(uint8_t x, uint8_t y, system_element_e systemElement)
{
    std::array<uint8_t, 4> cmd {
        COMMAND_SYS,
        x,
        y,
        systemElement
    };

    return output(MSP_DISPLAYPORT, &cmd[0], sizeof(cmd));
}

uint32_t DisplayPortMSP::txBytesFree() const
{
    //return mspSerialTxBytesFree();
    return 100;
}
