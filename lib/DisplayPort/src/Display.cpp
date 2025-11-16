#include "Display.h"
#include <cstring>

Display::Display(DisplayPortBase& displayPort) :
    _displayPort(displayPort)
{
}

void Display::init(DisplayPortBase::device_type_e deviceType)
{
    _displayPort.setDeviceType(deviceType);

    _displayPort._useFullscreen = false;
    _displayPort.releaseAll();

    beginTransaction(DISPLAY_TRANSACTION_OPTION_NONE);
    clearScreen(DISPLAY_CLEAR_WAIT);
    commitTransaction();
}

void Display::clearScreen(display_clear_option_e options)
{
    _displayPort.clearScreen(options);
    _displayPort._cleared = true;
    _displayPort._cursorRow = 255;
}

// Return true if screen still being transferred
bool Display::drawScreen()
{
    return _displayPort.drawScreen();
}

int Display::screenSize() const
{
    return _displayPort.screenSize();
}

void Display::grab()
{
    _displayPort.grab();
    _displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
    ++_displayPort._grabCount;
}

void Display::release()
{
    _displayPort.release();
    --_displayPort._grabCount;
}

void Display::releaseAll()
{
    _displayPort.release();
    _displayPort._grabCount = 0;
}

bool Display::isGrabbed() const
{
    // can be called before initialised
    return _displayPort._grabCount > 0;
}

void Display::setXY(uint8_t x, uint8_t y)
{
    _displayPort.setPosX(x);
    _displayPort.setPosY(y);
}

int Display::sys(uint8_t x, uint8_t y, DisplayPortBase::system_element_e systemElement)
{
    return _displayPort.writeSys(x, y, systemElement);
}

int Display::write(uint8_t x, uint8_t y, uint8_t attr, const char *text)
{
    _displayPort.setPosX(x + static_cast<uint8_t>(strlen(text)));
    _displayPort.setPosY(y);

    if (strlen(text) == 0) {
        // No point sending a message to do nothing
        return 0;
    }

    return _displayPort.writeString(x, y, attr, text);
}

int Display::writeChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    _displayPort.setPosX(x + 1);
    _displayPort.setPosY(y);
    return _displayPort.writeChar(x, y, attr, c);
}

bool Display::isTransferInProgress() const
{
    return _displayPort.isTransferInProgress();
}

bool Display::isSynced() const
{
    return _displayPort.isSynced();
}

bool Display::heartbeat()
{
    return _displayPort.heartbeat();
}

void Display::redraw()
{
    _displayPort.redraw();
}

uint32_t Display::txBytesFree() const
{
    return _displayPort.txBytesFree();
}

bool Display::layerSupported(DisplayPortBase::layer_e layer)
{
    if (layer == DisplayPortBase::LAYER_FOREGROUND) {
        // Every device must support the foreground (default) layer
        return true;
    }
    if (layer < DisplayPortBase::LAYER_COUNT) {
        return _displayPort.layerSupported(layer);
    }
    return false;
}

bool Display::layerSelect(DisplayPortBase::layer_e layer)
{
    return _displayPort.layerSelect(layer);
}

bool Display::layerCopy(DisplayPortBase::layer_e destLayer, DisplayPortBase::layer_e sourceLayer)
{
    if (sourceLayer != destLayer) {
        return _displayPort.layerCopy(destLayer, sourceLayer);
    }
    return false;
}

bool Display::writeFontCharacter(uint16_t addr, const osd_character_t* chr)
{
    return _displayPort.writeFontCharacter(addr, chr);
}

void Display::setBackgroundType(DisplayPortBase::background_e backgroundType)
{
    _displayPort.setBackgroundType(backgroundType);
}

bool Display::checkReady(bool rescan)
{
    return _displayPort.checkReady(rescan);
}

void Display::beginTransaction(display_transaction_option_e opts)
{
    _displayPort.beginTransaction(opts);
}

void Display::commitTransaction()
{
    _displayPort.commitTransaction();
}

bool Display::getCanvas(display_canvas_t* canvas) const
{
#if defined(USE_CANVAS)
    if (canvas && _displayPort.getCanvas && _displayPort.getCanvas(canvas, instance)) {
        canvas->gridElementWidth = canvas->width / _displayPort.cols;
        canvas->gridElementHeight = canvas->height / _displayPort.rows;
        return true;
    }
#else
    (void)canvas;
#endif
    return false;
}

bool Display::supportsOsdSymbols()
{
    // Assume device types that support OSD display will support the OSD symbols (since the OSD logic will use them)
    if ((_displayPort.getDeviceType() == DisplayPortBase::DEVICE_TYPE_MAX7456)
        || (_displayPort.getDeviceType() == DisplayPortBase::DEVICE_TYPE_MSP)
        || (_displayPort.getDeviceType() == DisplayPortBase::DEVICE_TYPE_FRSKY_OSD)) {
        return true;
    }
    return false;
}
