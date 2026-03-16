#pragma once

#include "display_port_base.h"

#if defined(M5_UNIFIED)
#include <M5GFX.h>
#include <M5Unified.h>
#endif


class DisplayPortM5GFX : public DisplayPortBase {
public:
#if defined(M5_UNIFIED)
    DisplayPortM5GFX(M5Canvas& _canvas, uint32_t screen_width_pixels, uint32_t screen_height_pixels);
#endif
    uint32_t clear_screen(display_clear_option_e options) override;
    bool draw_screen() override;
    void begin_transaction(display_transaction_option_e options) override;
    void commit_transaction() override;
    uint32_t tx_bytes_free() const override;
    uint32_t write_string(uint8_t x, uint8_t y, const char* text, uint8_t attr) override;
    uint32_t write_char(uint8_t x, uint8_t y, uint8_t c, uint8_t attr) override;
private:
    uint32_t _screen_width_pixels;
    uint32_t _screen_height_pixels;
    int32_t _x_scale {12};//{320/30}; //10
    int32_t _y_scale {20};//{240/16}; //15
    //int32_t _x_scale {320/53}; //6
    //int32_t _y_scale {240/20}; //12
#if defined(M5_UNIFIED)
    M5Canvas& _canvas;
#endif
    bool _inTransaction {false};
};
