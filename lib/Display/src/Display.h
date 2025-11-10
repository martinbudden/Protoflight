#pragma once

#include "DisplayPortBase.h"


class Display {
public:
    virtual ~Display() = default;
    explicit Display(DisplayPortBase& displayPort);
private:
    // Display is not copyable or moveable
    Display(const Display&) = delete;
    Display& operator=(const Display&) = delete;
    Display(Display&&) = delete;
    Display& operator=(Display&&) = delete;
public:
    void init(DisplayPortBase::device_type_e deviceType);
    void grab();
    void release();
    void releaseAll();
    bool isGrabbed() const;
    void clearScreen(display_clear_option_e options);
    bool drawScreen();
    int screenSize() const;
    void setXY(uint8_t x, uint8_t y);
    int sys(uint8_t x, uint8_t y, DisplayPortBase::system_element_e systemElement);
    int write(uint8_t x, uint8_t y, uint8_t attr, const char *text);
    int writeChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c);
    bool isTransferInProgress() const;
    bool heartbeat();
    void redraw();
    bool isSynced() const;
    uint32_t txBytesFree() const;
    bool writeFontCharacter(uint16_t addr, const struct osd_character_t* chr);
    bool checkReady(bool rescan);
    void beginTransaction(display_transaction_option_e opts);
    void commitTransaction();
    bool getCanvas(struct display_canvas_t* canvas) const;
    bool layerSupported(DisplayPortBase::layer_e layer);
    bool layerSelect(DisplayPortBase::layer_e layer);
    bool layerCopy(DisplayPortBase::layer_e destLayer, DisplayPortBase::layer_e sourceLayer);
    void setBackgroundType(DisplayPortBase::background_e backgroundType);
    bool supportsOsdSymbols();
private:
    DisplayPortBase& _displayPort;
};
