#include "display.h"
#include <cstring>

Display::Display(DisplayPortBase& display_port) :
    _display_port(display_port)
{
}

void Display::init(DisplayPortBase::device_type_e device_type)
{
    _display_port.set_device_type(device_type);

    _display_port.set_use_full_screen(false);
    _display_port.release_all();

    begin_transaction(DISPLAY_TRANSACTION_OPTION_NONE);
    clear_screen(DISPLAY_CLEAR_WAIT);
    commit_transaction();
}

void Display::clear_screen(display_clear_option_e options)
{
    _display_port.clear_screen(options);
    _display_port.set_cleared(true);
//    _display_port.setCursorRow(255);
}

// Return true if screen still being transferred
bool Display::draw_screen()
{
    return _display_port.draw_screen();
}

uint32_t Display::screen_size() const
{
    return _display_port.screen_size();
}

void Display::grab()
{
    _display_port.grab();
    _display_port.clear_screen(DISPLAY_CLEAR_WAIT);
    //++_display_port._grab_count;
}

void Display::release()
{
    _display_port.release();
    //--_display_port._grab_count;
}

void Display::release_all()
{
    _display_port.release_all();
}

bool Display::is_grabbed() const
{
    // can be called before initialised
    return _display_port.is_grabbed();
}

void Display::setXY(uint8_t x, uint8_t y)
{
    _display_port.set_pos_x(x);
    _display_port.set_pos_y(y);
}

uint32_t Display::sys(uint8_t x, uint8_t y, DisplayPortBase::system_element_e systemElement)
{
    return _display_port.write_sys(x, y, systemElement);
}

uint32_t Display::write(uint8_t x, uint8_t y, const char *text, uint8_t attr)
{
    _display_port.set_pos_x(x + static_cast<uint8_t>(strlen(text)));
    _display_port.set_pos_y(y);

    if (strlen(text) == 0) {
        // No point sending a message to do nothing
        return 0;
    }

    return _display_port.write_string(x, y, text, attr);
}

uint32_t Display::write_char(uint8_t x, uint8_t y, uint8_t c, uint8_t attr)
{
    _display_port.set_pos_x(x + 1);
    _display_port.set_pos_y(y);
    return _display_port.write_char(x, y, c, attr);
}

bool Display::is_transfer_in_progress() const
{
    return _display_port.is_transfer_in_progress();
}

bool Display::is_synced() const
{
    return _display_port.is_synced();
}

bool Display::heartbeat()
{
    return _display_port.heartbeat();
}

void Display::redraw()
{
    _display_port.redraw();
}

uint32_t Display::tx_bytes_free() const
{
    return _display_port.tx_bytes_free();
}

bool Display::is_layer_supported(DisplayPortBase::layer_e layer)
{
    if (layer == DisplayPortBase::LAYER_FOREGROUND) {
        // Every device must support the foreground (default) layer
        return true;
    }
    if (layer < DisplayPortBase::LAYER_COUNT) {
        return _display_port.is_layer_supported(layer);
    }
    return false;
}

bool Display::layer_select(DisplayPortBase::layer_e layer)
{
    return _display_port.layer_select(layer);
}

bool Display::layer_copy(DisplayPortBase::layer_e destLayer, DisplayPortBase::layer_e sourceLayer)
{
    if (sourceLayer != destLayer) {
        return _display_port.layer_copy(destLayer, sourceLayer);
    }
    return false;
}

bool Display::write_font_character(uint16_t addr, const osd_character_t* chr)
{
    return _display_port.write_font_character(addr, chr);
}

void Display::set_background_type(DisplayPortBase::background_e background_type)
{
    _display_port.set_background_type(background_type);
}

bool Display::check_ready(bool rescan)
{
    return _display_port.check_ready(rescan);
}

void Display::begin_transaction(display_transaction_option_e opts)
{
    _display_port.begin_transaction(opts);
}

void Display::commit_transaction()
{
    _display_port.commit_transaction();
}

bool Display::get_canvas(display_canvas_t* canvas) const
{
#if defined(USE_CANVAS)
    if (canvas && _display_port.get_canvas && _display_port.get_canvas(canvas, instance)) {
        canvas->gridElement_width = canvas->width / _display_port.cols;
        canvas->gridElementHeight = canvas->height / _display_port.rows;
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
    if ((_display_port.get_device_type() == DisplayPortBase::DEVICE_TYPE_MAX7456)
        || (_display_port.get_device_type() == DisplayPortBase::DEVICE_TYPE_MSP)
        || (_display_port.get_device_type() == DisplayPortBase::DEVICE_TYPE_FRSKY_OSD)) {
        return true;
    }
    return false;
}
