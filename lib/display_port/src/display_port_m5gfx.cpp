#include "display_port_m5gfx.h"
#include <cassert>

#if defined(M5_UNIFIED)


DisplayPortM5GFX::DisplayPortM5GFX(M5Canvas& canvas, uint32_t screen_width_pixels, uint32_t screen_height_pixels) :
    _screen_width_pixels(screen_width_pixels),
    _screen_height_pixels(screen_height_pixels),
    _canvas(canvas)
{
    _row_count = 15;
    _column_count = 26;
    _canvas.setColorDepth(1);
    if (!_canvas.createSprite(static_cast<int32_t>(_screen_width_pixels), static_cast<int32_t>(_screen_height_pixels))) {
        assert(false);
    }
    _canvas.fillSprite(TFT_WHITE);
    _canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    _canvas.setTextSize(2);
}

uint32_t DisplayPortM5GFX::tx_bytes_free() const
{
    return  _screen_width_pixels * _screen_height_pixels / (_x_scale * _y_scale); // 240 for 320*240 screen
}

uint32_t DisplayPortM5GFX::clear_screen(display_clear_option_e options)
{
    (void)options;
    _cleared = true;
    _canvas.fillSprite(TFT_WHITE);
    _canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    _canvas.setTextSize(2);
    return 0;
}

// Returns true if screen still being transferred
bool DisplayPortM5GFX::draw_screen()
{
    return false;
}

void DisplayPortM5GFX::begin_transaction(display_transaction_option_e option)
{
    //Serial.printf("begin_transaction\r\n");
    _inTransaction = true;
    _cleared = true; //!!FOR NOW
    if (option == DISPLAY_TRANSACTION_OPTION_RESET_DRAWING) {
        _canvas.fillSprite(TFT_WHITE);
        _canvas.setTextColor(TFT_BLACK, TFT_WHITE);
        _canvas.setTextSize(2);
    }
}

void DisplayPortM5GFX::commit_transaction()
{
    //Serial.printf("commit_transaction:%d\r\n", _inTransaction);
    if (_inTransaction) {
        _inTransaction = false;
        //_canvas.startWrite();
        _canvas.pushSprite(0, 0);
        //_canvas.endWrite();
    }
}

uint32_t DisplayPortM5GFX::write_string(uint8_t x, uint8_t y, const char *text, uint8_t attr)
{
    (void)attr;
    _canvas.setCursor(x*_x_scale, y*_y_scale);
    _canvas.print(text);
    //Serial.println(text);
    return 0;
}

uint32_t DisplayPortM5GFX::write_char(uint8_t x, uint8_t y, uint8_t c, uint8_t attr)
{
    (void)attr;
    std::array<char, 2> text = { c, 0 };
    _canvas.setCursor(x*_x_scale, y*_y_scale);
    _canvas.print(&text[0]);
    //Serial.printf("%c", c);
    return 0;
}

#endif // M5_UNIFIED
