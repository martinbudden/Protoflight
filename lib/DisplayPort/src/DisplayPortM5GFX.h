#pragma once

#include "DisplayPortBase.h"

#if defined(M5_UNIFIED)
#include <M5GFX.h>
#include <M5Unified.h>
#endif


class DisplayPortM5GFX : public DisplayPortBase {
public:
#if defined(M5_UNIFIED)
    DisplayPortM5GFX(M5Canvas& _canvas, uint32_t screenWidthPixels, uint32_t screenHeightPixels);
#endif
    int clearScreen(display_clear_option_e options) override;
    bool drawScreen() override;
    void beginTransaction(display_transaction_option_e options) override;
    void commitTransaction() override;
    int writeString(uint8_t x, uint8_t y, uint8_t attr, const char *text) override;
    int writeChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c) override;
private:
    uint32_t _screenWidthPixels;
    uint32_t _screenHeightPixels;
    int32_t _xScale {12};//{320/30}; //10
    int32_t _yScale {20};//{240/16}; //15
    //int32_t _xScale {320/53}; //6
    //int32_t _yScale {240/20}; //12
#if defined(M5_UNIFIED)
    M5Canvas& _canvas;
#endif
    bool _inTransaction {false};
};
