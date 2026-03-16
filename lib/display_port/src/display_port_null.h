#pragma once

#include "display_port_base.h"


class DisplayPortNull : public DisplayPortBase {
public:
    bool draw_screen() override { return 0; }
    uint32_t write_string(uint8_t x, uint8_t y, const char *text, uint8_t attr) override { (void)x; (void)y; (void)text; (void)attr; return 0; }
    uint32_t write_char(uint8_t x, uint8_t y, uint8_t c, uint8_t attr) override { (void)x; (void)y; (void)c; (void)attr; return 0; }
};
