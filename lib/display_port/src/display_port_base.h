#pragma once

#include <cstdint>

enum { VIDEO_COLUMNS_SD = 30 };
enum { VIDEO_LINES_NTSC = 13, VIDEO_LINES_PAL = 16 };

struct display_canvas_t;

enum display_transaction_option_e {
    DISPLAY_TRANSACTION_OPTION_NONE = 0x00,
    DISPLAY_TRANSACTION_OPTION_PROFILED = 0x01,
    DISPLAY_TRANSACTION_OPTION_RESET_DRAWING = 0x02,
};

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
public:
    static constexpr uint8_t BLINK = 0x80; // blink attribute bit

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
    virtual uint32_t clear_screen(display_clear_option_e options) { (void)options; _cleared = true; return 0; }
    virtual bool draw_screen() = 0; // Returns true if screen still being transferred

    virtual uint32_t write_string(uint8_t x, uint8_t y, const char *text, uint8_t attr) = 0;
    uint32_t write_string_normal(uint8_t x, uint8_t y, const char *text) { return write_string(x, y, text, SEVERITY_NORMAL); }
    //uint32_t write_string(uint8_t x, uint8_t y, const uint8_t* text, uint8_t attr) { return write_string(x, y, reinterpret_cast<const char*>(text), attr); }
    //uint32_t write_string(uint8_t x, uint8_t y, const uint8_t* text) { return write_string(x, y, reinterpret_cast<const char*>(text), SEVERITY_NORMAL); }

    virtual uint32_t write_char(uint8_t x, uint8_t y, uint8_t c, uint8_t attr) = 0;
    uint32_t write_char_normal(uint8_t x, uint8_t y, uint8_t c) { return write_char(x, y, c, SEVERITY_NORMAL); }

    virtual uint32_t screen_size() const { return _row_count * _column_count; }
    virtual uint32_t write_sys(uint8_t x, uint8_t y, system_element_e systemElement) { (void)x; (void)y; (void)systemElement; return 0; }
    virtual bool is_transfer_in_progress() const { return false; }
    virtual int heartbeat() { return 0; }
    virtual void redraw() { draw_screen(); }
    virtual bool is_synced() const {return true;}
    virtual uint32_t tx_bytes_free() const {return 0;}

    virtual bool is_layer_supported(layer_e layer) {
        if (layer == LAYER_FOREGROUND) { return true; } // Every device must support the foreground (default) layer
        return false;
    }
    virtual bool layer_select(layer_e layer) { if (is_layer_supported(layer)) { _active_layer = layer; return true; } return false; }
    virtual bool layer_copy(layer_e destLayer, layer_e sourceLayer) { return (sourceLayer == destLayer) ? false : true; }

    virtual bool write_font_character(uint16_t addr, const struct osd_character_t* chr) { (void)addr; (void)chr; return false; }
    virtual bool check_ready(bool rescan) { (void)rescan; return true; }
    virtual void begin_transaction(display_transaction_option_e option) { (void)option; }
    virtual void commit_transaction() {}
    virtual bool get_canvas(display_canvas_t* canvas) const { (void)canvas; return false; }
    virtual void set_background_type(background_e background_type) { (void)background_type; }

    virtual int grab() { clear_screen(DISPLAY_CLEAR_WAIT); ++_grab_count; return 0; }
    bool is_grabbed() const { return _grab_count > 0; }
    virtual uint32_t release() { --_grab_count; return 0; }
    void release_all() { _grab_count = 0; }

    bool is_cleared() const { return _cleared; }
    void set_cleared(bool cleared) { _cleared = cleared; }

    uint8_t get_row_count() const { return _row_count; }
    uint8_t get_column_count() const { return _column_count; }

    uint8_t get_pos_x() const { return _pos_x; }
    void set_pos_x(uint8_t pos_x) { _pos_x = pos_x; }
    uint8_t get_pos_y() const { return _pos_y; }
    void set_pos_y(uint8_t pos_y) { _pos_y = pos_y; }

    uint8_t get_small_arrow_up() const { return _small_arrow_up; }
    uint8_t get_small_arrow_down() const { return _small_arrow_down; }
    bool get_use_full_screen() const { return _use_full_screen; }
    void set_use_full_screen(bool use_full_screen) { _use_full_screen = use_full_screen; }

    device_type_e get_device_type() const { return _device_type; }
    void set_device_type(device_type_e device_type) { _device_type = device_type; }
    bool get_use_device_blink() const { return _use_device_blink; }

protected:
    device_type_e _device_type {};
    background_e _background_type {BACKGROUND_TRANSPARENT};
    layer_e _active_layer {LAYER_FOREGROUND};
    uint16_t _max_screen_size {};

    uint8_t _row_count {};
    uint8_t _column_count {};
    uint8_t _pos_x {};
    uint8_t _pos_y {};
    uint8_t _small_arrow_up {'^'};
    uint8_t _small_arrow_down {'v'};

    // Displayport device capability
    bool _use_device_blink {};
    // CMS state
    int8_t _grab_count {0};
    bool _cleared {};
    bool _use_full_screen {false}; // tru for DEVICE_TYPE_HOTT, false otherwise
};
