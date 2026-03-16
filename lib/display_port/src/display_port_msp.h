#pragma once

#include "display_port_base.h"

class MspStream;


class DisplayPortMSP : public DisplayPortBase {
public:
    // MSP Display Port commands
    enum {
        COMMAND_HEARTBEAT = 0,         // Release the display after clearing and updating
        COMMAND_RELEASE = 1,         // Release the display after clearing and updating
        COMMAND_CLEAR_SCREEN = 2,    // Clear the display
        COMMAND_WRITE_STRING = 3,    // Write a string at given coordinates
        COMMAND_DRAW_SCREEN = 4,     // Trigger a screen draw
        COMMAND_OPTIONS = 5,         // Not used by Betaflight. Reserved by Ardupilot and INAV
        COMMAND_SYS = 6,             // Display system element displayportSystemElement_e at given coordinates
        COMMAND_COUNT,
    };
    enum { MSP_DISPLAYPORT = 182 };  // out message: External OSD displayport mode
    // MSP displayport V2 attribute byte bit flags
    static constexpr uint8_t ATTR_VERSION = 0b10000000; // BIT(7) Format indicator; must be zero for V2 and V1
    static constexpr uint8_t ATTR_BLINK   = 0b01000000; // BIT(6) Device local blink
    static constexpr uint8_t ATTR_FONT    = 0b00000011; // (BIT(0) | BIT(1)) Select bank of 256 characters as per severity
    static constexpr uint8_t ATTR_MASK    = ATTR_VERSION | ATTR_BLINK | ATTR_FONT;
public:
    DisplayPortMSP(MspStream& msp_stream);
    uint32_t clear_screen(display_clear_option_e options) override;
    bool draw_screen() override;
    uint32_t write_string(uint8_t x, uint8_t y, const char* text, uint8_t attr) override;
    uint32_t write_char(uint8_t x, uint8_t y, uint8_t c, uint8_t attr) override;
    uint32_t write_sys(uint8_t x, uint8_t y, system_element_e systemElement) override;
    uint32_t tx_bytes_free() const override;
    int heartbeat() override;
    int grab() override;
    uint32_t release() override;
private:
    uint32_t output(const uint8_t *buf, uint8_t len);
    uint32_t output(uint8_t sub_command);
private:
    MspStream& _msp_stream;
    bool _inTransaction {false};
};
