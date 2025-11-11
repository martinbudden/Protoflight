#pragma once

#include "DisplayPortBase.h"

#if defined(M5_UNIFIED)
#include <M5Unified.h>
#endif


class DisplayPortM5GFX : public DisplayPortBase {
public:
    DisplayPortM5GFX();
    int clearScreen(display_clear_option_e options) override;
    bool drawScreen() override;
    int writeString(uint8_t x, uint8_t y, uint8_t attr, const char *text) override;
    int writeChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c) override;
private:
    int32_t _xScale {8};
    int32_t _yScale {8};
#if defined(M5_UNIFIED)
    M5Canvas& _canvas;
#endif
};
