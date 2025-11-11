#include "DisplayPortM5GFX.h"

#if defined(M5_UNIFIED)

static M5Canvas canvas(&M5.Display);

DisplayPortM5GFX::DisplayPortM5GFX() :
    _canvas(canvas)
{
    _canvas.fillSprite(TFT_WHITE);
    _canvas.setTextSize(1);
}

int DisplayPortM5GFX::clearScreen(display_clear_option_e options)
{ 
    (void)options;
    cleared = true;
    cursorRow = -1;
    _canvas.fillSprite(TFT_WHITE);
    //_canvas.startWrite();
    //_canvas.pushSprite(0, 0);
    //_canvas.endWrite();
    return 0; 
}

bool DisplayPortM5GFX::drawScreen()
{
    _canvas.startWrite();
    _canvas.pushSprite(0, 0);
    _canvas.endWrite();
    return true;
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
