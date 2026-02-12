#include "DisplayPortM5GFX.h"
#include <cassert>

#if defined(M5_UNIFIED)


DisplayPortM5GFX::DisplayPortM5GFX(M5Canvas& canvas, uint32_t screenWidthPixels, uint32_t screenHeightPixels) :
    _screenWidthPixels(screenWidthPixels),
    _screenHeightPixels(screenHeightPixels),
    _canvas(canvas)
{
    _rowCount = 15;
    _columnCount = 26;
    _canvas.setColorDepth(1);
    if (!_canvas.createSprite(static_cast<int32_t>(_screenWidthPixels), static_cast<int32_t>(_screenHeightPixels))) {
        assert(false);
    }
    _canvas.fillSprite(TFT_WHITE);
    _canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    _canvas.setTextSize(2);
}

uint32_t DisplayPortM5GFX::txBytesFree() const
{
    return  _screenWidthPixels * _screenHeightPixels / (_xScale * _yScale); // 240 for 320*240 screen
}

uint32_t DisplayPortM5GFX::clearScreen(display_clear_option_e options)
{
    (void)options;
    _cleared = true;
    _canvas.fillSprite(TFT_WHITE);
    _canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    _canvas.setTextSize(2);
    return 0;
}

// Returns true if screen still being transferred
bool DisplayPortM5GFX::drawScreen()
{
    return false;
}

void DisplayPortM5GFX::beginTransaction(display_transaction_option_e option)
{
    //Serial.printf("beginTransaction\r\n");
    _inTransaction = true;
    _cleared = true; //!!FOR NOW
    if (option == DISPLAY_TRANSACTION_OPTION_RESET_DRAWING) {
        _canvas.fillSprite(TFT_WHITE);
        _canvas.setTextColor(TFT_BLACK, TFT_WHITE);
        _canvas.setTextSize(2);
    }
}

void DisplayPortM5GFX::commitTransaction()
{
    //Serial.printf("commitTransaction:%d\r\n", _inTransaction);
    if (_inTransaction) {
        _inTransaction = false;
        //_canvas.startWrite();
        _canvas.pushSprite(0, 0);
        //_canvas.endWrite();
    }
}

uint32_t DisplayPortM5GFX::writeString(uint8_t x, uint8_t y, const char *text, uint8_t attr)
{
    (void)attr;
    _canvas.setCursor(x*_xScale, y*_yScale);
    _canvas.print(text);
    //Serial.println(text);
    return 0;
}

uint32_t DisplayPortM5GFX::writeChar(uint8_t x, uint8_t y, uint8_t c, uint8_t attr)
{
    (void)attr;
    std::array<char, 2> text = { c, 0 };
    _canvas.setCursor(x*_xScale, y*_yScale);
    _canvas.print(&text[0]);
    //Serial.printf("%c", c);
    return 0;
}

#endif // M5_UNIFIED
