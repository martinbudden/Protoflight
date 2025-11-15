#include "DisplayPortM5GFX.h"
//#include <HardwareSerial.h>

#if defined(M5_UNIFIED)


DisplayPortM5GFX::DisplayPortM5GFX(M5Canvas& canvas, uint32_t screenWidthPixels, uint32_t screenHeightPixels) :
    _screenWidthPixels(screenWidthPixels),
    _screenHeightPixels(screenHeightPixels),
    _canvas(canvas)
{
    _rowCount = 16;
    _columnCount = 26;
    _canvas.fillSprite(TFT_WHITE);
    _canvas.setTextColor(TFT_BLACK);
    _canvas.setTextSize(2);
}

int DisplayPortM5GFX::clearScreen(display_clear_option_e options)
{ 
    (void)options;
    _cleared = true;
    _cursorRow = -1;
    _canvas.fillSprite(TFT_WHITE);
    _canvas.setTextColor(TFT_BLACK);
    _canvas.setTextSize(2);
    //_canvas.startWrite();
    //_canvas.pushSprite(0, 0);
    //_canvas.endWrite();
    return 0; 
}

bool DisplayPortM5GFX::drawScreen()
{
    return true;
}

void DisplayPortM5GFX::beginTransaction(display_transaction_option_e options)
{
    (void)options;
    //Serial.printf("beginTransaction\r\n");
    _inTransaction = true;
    _canvas.createSprite(_screenWidthPixels, _screenHeightPixels);

}

void DisplayPortM5GFX::commitTransaction()
{
    //Serial.printf("commitTransaction:%d\r\n", _inTransaction);
    if (_inTransaction) {
        _canvas.startWrite();
        _canvas.pushSprite(0, 0);
        _canvas.endWrite();
        _inTransaction = false;
    }
}

int DisplayPortM5GFX::writeString(uint8_t x, uint8_t y, uint8_t attr, const char *text)
{
    (void)attr;
    _canvas.setCursor(x*_xScale, y*_yScale);
    _canvas.print(text);
    return 0;
}

int DisplayPortM5GFX::writeChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    (void)attr;
    std::array<char, 2> text = { c, 0 };
    _canvas.setCursor(x*_xScale, y*_yScale);
    _canvas.print(&text[0]);
    return 0;
}

#endif // M5_UNIFIED
