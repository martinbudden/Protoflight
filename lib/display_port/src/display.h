#pragma once

#include "display_port_base.h"


class Display {
public:
    virtual ~Display() = default;
    explicit Display(DisplayPortBase& display_port);
private:
    // Display is not copyable or moveable
    Display(const Display&) = delete;
    Display& operator=(const Display&) = delete;
    Display(Display&&) = delete;
    Display& operator=(Display&&) = delete;
public:
    void init(DisplayPortBase::device_type_e device_type);
    void grab();
    void release();
    void release_all();
    bool is_grabbed() const;
    void clear_screen(display_clear_option_e options);
    bool draw_screen();
    uint32_t screen_size() const;
    void setXY(uint8_t x, uint8_t y);
    uint32_t sys(uint8_t x, uint8_t y, DisplayPortBase::system_element_e systemElement);
    uint32_t write(uint8_t x, uint8_t y, const char *text, uint8_t attr);
    uint32_t write_char(uint8_t x, uint8_t y, uint8_t c, uint8_t attr);
    bool is_transfer_in_progress() const;
    bool heartbeat();
    void redraw();
    bool is_synced() const;
    uint32_t tx_bytes_free() const;
    bool write_font_character(uint16_t addr, const struct osd_character_t* chr);
    bool check_ready(bool rescan);
    void begin_transaction(display_transaction_option_e opts);
    void commit_transaction();
    bool get_canvas(struct display_canvas_t* canvas) const;
    bool is_layer_supported(DisplayPortBase::layer_e layer);
    bool layer_select(DisplayPortBase::layer_e layer);
    bool layer_copy(DisplayPortBase::layer_e destLayer, DisplayPortBase::layer_e sourceLayer);
    void set_background_type(DisplayPortBase::background_e background_type);
    bool supportsOsdSymbols();
private:
    DisplayPortBase& _display_port;
};
