#pragma once

#include "DisplayPortBase.h"


class DisplayPortNull : public DisplayPortBase {
public:
    bool drawScreen() override { return 0; }
    uint32_t writeString(uint8_t x, uint8_t y, uint8_t attr, const char *text) override { (void)x; (void)y; (void)attr; (void)text; return 0; }
    uint32_t writeChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c) override { (void)x; (void)y; (void)attr; (void)c; return 0; }
};
