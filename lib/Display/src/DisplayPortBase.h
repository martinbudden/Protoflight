#pragma once

#include <cstdint>

enum { VIDEO_COLUMNS_SD = 30 };
enum { VIDEO_LINES_NTSC = 13, VIDEO_LINES_PAL = 16 };

struct display_canvas_t;
struct osdCharacter_t;

enum display_transaction_option_e {
    DISPLAY_TRANSACTION_OPT_NONE = 0x00,
    DISPLAY_TRANSACTION_OPT_PROFILED = 0x01,
    DISPLAY_TRANSACTION_OPT_RESET_DRAWING = 0x02,
} ;

enum display_clear_option_e {
    // Display drivers that can perform screen clearing in the background, e.g. via DMA, should do so.
    // use `displayCheckReady` function to check if the screen clear has been completed.
    DISPLAY_CLEAR_NONE = 0,

    // * when set, the display driver should block until the screen clear has completed, use in synchronous cases
    //   only, e.g. where the screen is cleared and the display is immediately drawn to.
    // * when NOT set, return immediately and do not block unless screen is a simple operation or cannot
    //   be performed in the background.  As with any long delay, waiting can cause task starvation which
    //   can result in RX loss.
    DISPLAY_CLEAR_WAIT = 0x01,
};

class DisplayPortBase {
friend class Display; //!!FOR NOW
public:
    enum { BLINK = 0x80 }; // blink attribute bit

    enum device_type_e {
        DEVICE_TYPE_NONE = 0,
        DEVICE_TYPE_AUTO,
        DEVICE_TYPE_MAX7456,
        DEVICE_TYPE_OLED,
        DEVICE_TYPE_FRSKY_OSD,
        DEVICE_TYPE_MSP,
        DEVICE_TYPE_CRSF,
        DEVICE_TYPE_HOTT,
        DEVICE_TYPE_SRXL,
    };
    enum severity_e {
        SEVERITY_NORMAL = 0,
        SEVERITY_INFO,
        SEVERITY_WARNING,
        SEVERITY_CRITICAL,
        SEVERITY_COUNT,
    };
    // System elements rendered by VTX or Goggles
    enum system_element_e {
        SYS_GOGGLE_VOLTAGE = 0,
        SYS_VTX_VOLTAGE = 1,
        SYS_BITRATE = 2,
        SYS_DELAY = 3,
        SYS_DISTANCE = 4,
        SYS_LQ = 5,
        SYS_GOGGLE_DVR = 6,
        SYS_VTX_DVR = 7,
        SYS_WARNINGS = 8,
        SYS_VTX_TEMP = 9,
        SYS_FAN_SPEED = 10,
        SYS_COUNT,
    };
    enum layer_e {
        LAYER_FOREGROUND,
        LAYER_BACKGROUND,
        LAYER_COUNT,
    };
    enum background_e {
        BACKGROUND_TRANSPARENT,
        BACKGROUND_BLACK,
        BACKGROUND_GRAY,
        BACKGROUND_LIGHT_GRAY,
        BACKGROUND_COUNT    // must be the last entry
    };
public:
    virtual ~DisplayPortBase() = default;
    DisplayPortBase() = default;
    virtual int clearScreen(display_clear_option_e options) { (void)options; _cleared = true; _cursorRow = -1; return 0; }
    virtual bool drawScreen() = 0;
    virtual int writeString(uint8_t x, uint8_t y, uint8_t attr, const char *text) = 0;
    virtual int writeChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c) = 0;

    virtual int grab() { return 0; }
    virtual int release() { return 0; }
    virtual int screenSize() const { return 0; }
    virtual int writeSys(uint8_t x, uint8_t y, system_element_e systemElement) { (void)x; (void)y; (void)systemElement; return 0; }
    virtual bool isTransferInProgress() const { return false; }
    virtual int heartbeat() { return 0; }
    virtual void redraw() {}
    virtual bool isSynced() const {return true;}
    virtual uint32_t txBytesFree() const {return 0;}
    virtual bool layerSupported(layer_e layer) { (void)layer; return false; }
    virtual bool layerSelect(layer_e layer) { (void)layer; return false; }
    virtual bool layerCopy(layer_e destLayer, layer_e sourceLayer) { return (sourceLayer == destLayer) ? false : true; }
    virtual bool writeFontCharacter(uint16_t addr, const struct osd_character_t* chr) { (void)addr; (void)chr; return false; }
    virtual bool checkReady(bool rescan) { (void)rescan; return true; }
    virtual void beginTransaction(display_transaction_option_e opts) {(void)opts;}
    virtual void commitTransaction() {}
    virtual bool getCanvas(display_canvas_t* canvas) const { (void)canvas; return false; }
    virtual void setBackgroundType(background_e backgroundType) { (void)backgroundType; }

    uint8_t getRowCount() const { return _rows; }
    uint8_t getColumnCount() const { return _cols; }

    uint8_t getPosX() const { return _posX; }
    void setPosX(uint8_t posX) { _posX = posX; }
    uint8_t getPosY() const { return _posY; }
    void setPosY(uint8_t posY) { _posY = posY; }

    void resetGrabCount() { _grabCount = 0; }

    device_type_e getDeviceType() const { return _deviceType; }
    void setDeviceType(device_type_e deviceType) { _deviceType = deviceType; }
    bool getUseDeviceBlink() const { return _useDeviceBlink; }
protected:
    device_type_e _deviceType {};

    uint8_t _rows {};
    uint8_t _cols {};
    uint8_t _posX {};
    uint8_t _posY {};

    // Displayport device capability
    bool _useDeviceBlink {};
    // CMS state
    int8_t _cursorRow {};
    int8_t _grabCount {};
    bool _cleared {};
    bool _useFullscreen {};
};
