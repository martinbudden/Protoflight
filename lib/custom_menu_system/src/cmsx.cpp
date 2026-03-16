#include "cms.h"
#include "cms_types.h"
#include "cmsx.h"
#include "cockpit.h"
#include "display_port_base.h"
#include "format_integer.h"
#include "osd_elements.h"

#include <algorithm>
#include <cstring>
#if (__cplusplus >= 202002L)
#include <ranges>
#endif


//NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
const CMSX::menu_t* CMSX::MENU_NULL_PTR          = nullptr;
const CMSX::menu_t* CMSX::MENU_EXIT              = CMSX::MENU_NULL_PTR + 1;
const CMSX::menu_t* CMSX::MENU_EXIT_SAVE         = CMSX::MENU_NULL_PTR + 2;
const CMSX::menu_t* CMSX::MENU_EXIT_SAVE_REBOOT  = CMSX::MENU_NULL_PTR + 3;
const CMSX::menu_t* CMSX::MENU_POPUP_SAVE        = CMSX::MENU_NULL_PTR + 4;
const CMSX::menu_t* CMSX::MENU_POPUP_SAVE_REBOOT = CMSX::MENU_NULL_PTR + 5;
const CMSX::menu_t* CMSX::MENU_POPUP_EXIT_REBOOT = CMSX::MENU_NULL_PTR + 6;
const CMSX::menu_t* CMSX::MENU_BACK              = CMSX::MENU_NULL_PTR + 7;
//NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)


CMSX::CMSX(CMS& cms) :
    _cms(cms),
    _menu_main(menu_main)
    //_menu_main(menu_filters)
    //_menu_main(menu_rates)
    //_menu_main(menu_rc_preview)
{
}

void CMSX::set_reboot_required()
{
    _reboot_required = true;
}

bool CMSX::get_reboot_required() const
{
    return _reboot_required;
}

CMSX::menu_t* CMSX::get_save_exit_menu() const
{
    if (_reboot_required) {
        return &CMSX::menu_save_exit_reboot;
    }
    return &CMSX::menu_save_exit;
}

bool CMSX::setup_popup_menu_build(cms_context_t& ctx) // NOLINT(readability-make-member-function-const)
{
    uint8_t menu_index = 0;

    menu_setup_popupEntries[menu_index] = { "-- SETUP MENU --", OME_LABEL, nullptr, nullptr };
    // Add menu entries for uncompleted setup tasks
    if (ctx.cockpit.get_arming_disabled_flags() & ARMING_DISABLED_ACC_CALIBRATION) { // NOLINT(hicpp-signed-bitwise)
        menu_setup_popupEntries[++menu_index] = { "CALIBRATE ACC", OME_FUNCTION_CALL | OME_DYNAMIC, menu_calibrate_acc, &AccCalibrationStatus[0] };
    }
#if defined(USE_BATTERY_CONTINUE)
    if (hasUsedMAh()) {
        cmsAddMenuEntry(&setupPopupMenuEntries[++menu_index], batteryContinueAmount, OME_Funcall | DYNAMIC, cmsRestoreMah, nullptr);
    }
#endif
    menu_setup_popupEntries[++menu_index] = { "EXIT", OME_BACK | OME_DYNAMIC, nullptr, nullptr };
    menu_setup_popupEntries[++menu_index] = { "NULL", OME_END, nullptr, nullptr };

    return (menu_index > 2);  // return true if any setup items were added
}

// Check if overridden by slider, used by simplified PID tuning
bool CMSX::row_slider_override(const uint16_t flags)
{
    (void)flags;
    return false;
}

bool CMSX::row_is_skippable(const OSD_Entry* row)
{
    const uint16_t entryType = row->flags & OME_TYPE_MASK;

    if (entryType == OME_LABEL) {
        return true;
    }
    if (entryType == OME_STRING) {
        return true;
    }
    if ((row->flags == OME_DYNAMIC) || row_slider_override(row->flags)) { // cppcheck-suppress knownConditionTrueFalse
        if (entryType == OME_UINT8  || entryType == OME_INT8 || entryType == OME_UINT16  || entryType == OME_INT16) {
            return true;
        }
    }
    return false;
}

// Pad buffer to the right, i.e. align left
void CMSX::pad_right(char *buf, uint8_t size)
{
    uint8_t ii = 0;
    while (ii < size) {
        if (buf[ii] == 0) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            break;
        }
        ++ii;
    }
    while (ii < size) {
        buf[ii] = ' '; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        ++ii;
    }
    buf[size] = 0; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

// Pad buffer to the left, i.e. align right
void CMSX::pad_left(char *buf, uint8_t size)
{
    auto len = static_cast<int32_t>(std::min(strlen(buf), size_t{size}));

    int32_t ii = size - 1;
    const int32_t jj = size - len;
    while (ii - jj >= 0) {
        buf[ii] = buf[ii - jj]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        --ii;
    }
    while (ii >= 0) {
        buf[ii] = ' '; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        --ii;
    }
    buf[size] = 0; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

void CMSX::pad_to_size(char* buf, uint8_t size) const
{
    if (_right_aligned) {
        pad_left(buf, size);
    } else {
        pad_right(buf, size);
    }
}

uint32_t CMSX::draw_menuItemValue(DisplayPortBase& display_port, uint8_t row, uint8_t maxSize) // NOLINT(readability-make-member-function-const)
{
    pad_to_size(&_menu_draw_buf[0], maxSize < MENU_DRAW_BUFFER_LEN ? maxSize : MENU_DRAW_BUFFER_LEN);
    const uint8_t column = _right_aligned ? _right_menu_column - maxSize : _right_menu_column;
    return display_port.write_string(column, row, &_menu_draw_buf[0]);
}

uint32_t CMSX::draw_menuTableItemValue(DisplayPortBase& display_port, uint8_t row, uint8_t maxSize) // NOLINT(readability-make-member-function-const)
{
    pad_to_size(&_menu_table_buf[0], maxSize < MENU_TABLE_BUFFER_LEN ? maxSize : MENU_TABLE_BUFFER_LEN);
    const uint8_t column = _right_aligned ? _right_menu_column - maxSize : _right_menu_column;
    return display_port.write_string(column, row, &_menu_table_buf[0]);
}

uint32_t CMSX::draw_menuTableEntry(DisplayPortBase& display_port, const OSD_Entry* entry, uint8_t row, uint16_t& entry_flags, table_ticker_t& ticker)
{
    uint32_t count = 0;

    if (((entry_flags & OME_PRINT_VALUE) || (entry_flags & OME_SCROLLING_TICKER)) && entry->data) {
        const auto* ptr = reinterpret_cast<const osd_table_t*>(entry->data); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        //const size_t labelLength = strnlen(entry->text , MENU_DRAW_BUFFER_LEN) + 1; // account for the space between label and display data
        const uint8_t index = std::clamp(*ptr->val, static_cast<uint8_t>(0), ptr->max);
        const char* str = static_cast<const char *>(ptr->names[index]);   // lookup table display text NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        const uint8_t displayLength = static_cast<uint8_t>(std::strlen(str));
        const uint8_t availableSpace = display_port.get_column_count() - _right_menu_column;

        bool drawText = false;
        if (entry_flags & OME_PRINT_VALUE) {
            drawText = true;
            ticker.state = 0;
            ticker.loop_counter = 0;
            if (displayLength > availableSpace) { // table entry text is longer than the available space so start the ticker
                set_flag(entry_flags, OME_SCROLLING_TICKER);
            } else {
                clear_flag(entry_flags, OME_SCROLLING_TICKER);
            }
        } else {
            ++ticker.loop_counter;
            const uint8_t loopLimit = (ticker.state == 0 || ticker.state == (displayLength - availableSpace)) ? LOOKUP_TABLE_TICKER_START_CYCLES : LOOKUP_TABLE_TICKER_SCROLL_CYCLES;
            if (ticker.loop_counter >= loopLimit) {
                ticker.loop_counter = 0;
                drawText = true;
                ++ticker.state;
                if (ticker.state > (displayLength - availableSpace)) {
                    ticker.state = 0;
                }
            }
        }
        if (drawText) {
            strncpy(&_menu_table_buf[0], static_cast<const char *>(str + ticker.state), MENU_TABLE_BUFFER_LEN);
            count += draw_menuTableItemValue(display_port, row, availableSpace);
        }
        clear_flag(entry_flags, OME_PRINT_VALUE);
    }

    return count;
}

uint32_t CMSX::draw_menuEntry(cms_context_t& ctx, const OSD_Entry* entry, uint8_t row, uint16_t& entry_flags, table_ticker_t& ticker) // NOLINT(readability-function-cognitive-complexity)
{
    const uint16_t entryType = entry->flags & OME_TYPE_MASK;
    uint32_t count = 0;
    if (entry_flags & OME_PRINT_LABEL) {
        const uint8_t column = _left_menu_column + ((entryType == OME_LABEL) ? 0 : 1);
        count += ctx.display_port.write_string(column, row, entry->text);
        clear_flag(entry_flags, OME_PRINT_LABEL);
    }

    if (entryType == OME_TABLE) {
        count += draw_menuTableEntry(ctx.display_port, entry, row, entry_flags, ticker);
        return count;
    };

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-pointer-arithmetic)
    switch (entryType) {
    case OME_LABEL:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            // A label with optional string, immediately following text
            strncpy(&_menu_draw_buf[0], reinterpret_cast<const char*>(entry->data), MENU_DRAW_BUFFER_LEN);
            count += draw_menuItemValue(ctx.display_port, row, static_cast<uint8_t>(std::min(strlen(&_menu_draw_buf[0]), size_t{MENU_DRAW_BUFFER_LEN})));
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_SUBMENU:
        [[fallthrough]];
    case OME_FUNCTION_CALL:
        if ((entry_flags & OME_PRINT_VALUE)) {
            _menu_draw_buf[0] = 0;
            if (entryType == OME_SUBMENU && entry->fnPtr && entry_flags & OME_OPTION_STRING) {
                // Special case of sub menu entry with optional value display.
                const char *str = reinterpret_cast<const char*>(entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data)));
                strncpy(&_menu_draw_buf[0], str, MENU_DRAW_BUFFER_LEN);
            } else if (entryType == OME_FUNCTION_CALL && entry->data) {
                strncpy(&_menu_draw_buf[0], reinterpret_cast<const char*>(entry->data), MENU_DRAW_BUFFER_LEN);
            }
            strncat(&_menu_draw_buf[0], ">", MENU_DRAW_BUFFER_LEN);
            row = _small_screen ? row - 1 : row;
            count += draw_menuItemValue(ctx.display_port, row, static_cast<uint8_t>(std::min(strlen(&_menu_draw_buf[0]), size_t{MENU_DRAW_BUFFER_LEN})));
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_STRING:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            strncpy(reinterpret_cast<char*>(&_menu_draw_buf[0]), static_cast<const char*>(entry->data), MENU_DRAW_BUFFER_LEN);
            count += draw_menuItemValue(ctx.display_port, row, MENU_DRAW_BUFFER_LEN);
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_BOOL:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const osd_bool_t*>(entry->data);
            _menu_draw_buf[0] = *ptr->val ? '1' : '0';
            _menu_draw_buf[1] = 0;
            count += draw_menuItemValue(ctx.display_port, row, NUMBER_FIELD_LEN);
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_UINT8:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const osd_uint8_t*>(entry->data);
            ui2a(*ptr->val, &_menu_draw_buf[0]);
            count += draw_menuItemValue(ctx.display_port, row, NUMBER_FIELD_LEN);
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_UINT8_FIXED:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const osd_uint8_fixed_t*>(entry->data);
            formatFixed6point3(*ptr->val*ptr->multiplier, &_menu_draw_buf[0]);
            ui2a(*ptr->val, &_menu_draw_buf[0]);
            count += draw_menuItemValue(ctx.display_port, row, NUMBER_FIELD_LEN);
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_INT8:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const osd_int8_t*>(entry->data);
            i2a(*ptr->val, &_menu_draw_buf[0]);
            count += draw_menuItemValue(ctx.display_port, row, NUMBER_FIELD_LEN);
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_UINT16:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const osd_uint16_t*>(entry->data);
            ui2a(*ptr->val, &_menu_draw_buf[0]);
            count += draw_menuItemValue(ctx.display_port, row, NUMBER_FIELD_LEN);
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_UINT16_FIXED:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const osd_uint16_fixed_t*>(entry->data);
            formatFixed6point3(*ptr->val*ptr->multiplier, &_menu_draw_buf[0]);
            ui2a(*ptr->val, &_menu_draw_buf[0]);
            count += draw_menuItemValue(ctx.display_port, row, NUMBER_FIELD_LEN);
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_INT16:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const osd_int16_t*>(entry->data);
            i2a(*ptr->val, &_menu_draw_buf[0]);
            count += draw_menuItemValue(ctx.display_port, row, NUMBER_FIELD_LEN);
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_UINT32:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const osd_uint32_t*>(entry->data);
            ui2a(*ptr->val, &_menu_draw_buf[0]);
            count += draw_menuItemValue(ctx.display_port, row, NUMBER_FIELD_LEN);
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    case OME_INT32:
        if ((entry_flags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const osd_int32_t*>(entry->data);
            i2a(*ptr->val, &_menu_draw_buf[0]);
            count += draw_menuItemValue(ctx.display_port, row, NUMBER_FIELD_LEN);
            clear_flag(entry_flags, OME_PRINT_VALUE);
        }
        break;
    default:
        break;
    }
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return count;
}

void CMSX::draw_menu(cms_context_t& ctx, uint32_t current_time_us) // NOLINT(readability-function-cognitive-complexity)
{
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    if (!_page_top || !_in_menu) {
        return;
    }
    const bool displayWasCleared = ctx.display_port.is_cleared();
    ctx.display_port.set_cleared(false);

    bool drawDynamicValues = false;
    if (current_time_us > _last_polled_us + DYNAMIC_VALUES_POLLING_INTERVAL_US) {
        drawDynamicValues = true;
        _last_polled_us = current_time_us;
    }

    if (displayWasCleared) {
        //Serial.printf("draw_menu setting entry_flags\r\n"); // called
#if (__cplusplus >= 202002L) && false
        // Iterate over indices [0, _page_max_row)
        for (auto ii : std::views::iota(size_t{0}, std::span(_page_top, _page_max_row).size())) {
            _entry_flags[ii] |= OME_PRINT_LABEL | OME_PRINT_VALUE;
        }
#else
        uint8_t ii = 0;
        for (const OSD_Entry* entry = _page_top; entry <= _page_top + _page_max_row; ++entry, ++ii) {
            _entry_flags[ii] |= OME_PRINT_LABEL | OME_PRINT_VALUE;
        }
#endif
#if false
    auto view = std::ranges::subrange(_page_top, _page_top + _page_max_row);

    // Iterate using ranges
    for (auto&& entry : view) {
        if (entry.text) {
            std::cout << "Text: " << entry.text
                      << " | Flags: " << entry.flags << "\n";
        }
        if (entry.fnPtr) {
            entry.fnPtr(entry.data);
        }
    }
}
#endif
    } else if (drawDynamicValues) {
        //Serial.printf("draw_menu drawDynamicValues\r\n");
#if (__cplusplus >= 202002L) && false
        auto entries = std::span(_page_top, _page_max_row);
        // Iterate over indices [0, _page_max_row)
        for (auto ii : std::views::iota(size_t{0}, entries.size())) {
            if (entries[ii].flags & OME_DYNAMIC) {
                _entry_flags[ii] |= OME_PRINT_VALUE;
            }
        }
#if false
        auto indexed = std::views::iota(size_t{0}, entries.size())
                 | std::views::transform([&](size_t i) { return std::pair{i, entries[i]}; });
        for (auto [i, entry] : indexed) {
            if (entry.flags & OME_DYNAMIC) {
                _entry_flags[i] |= OME_PRINT_VALUE;
            }
        }
    // pure one-liner C++20 ranges pipeline without explicit for loop
    // Still uses std::views::iota for indices, std::span for safe pointer-to-range conversion, and std::ranges::for_each to apply the update.
    std::ranges::for_each(
        std::views::iota(size_t{0}, entries.size())
        | std::views::filter([&](size_t i) { return entries[i].flags & OME_DYNAMIC; }),
        [&](size_t i) { _entry_flags[i] |= OME_PRINT_VALUE; }
    );
    // C++23 only
    std::ranges::for_each(
        std::views::zip(_entry_flags, entries)
        | std::views::filter([](auto&& pair) { return std::get<1>(pair).flags & OME_DYNAMIC; }),
        [](auto&& pair) { std::get<0>(pair) |= OME_PRINT_VALUE; }
    );
#endif
#else
        uint8_t ii = 0;
        for (const OSD_Entry* entry = _page_top; entry <= _page_top + _page_max_row; ++entry, ++ii) {
            if (entry->flags & OME_DYNAMIC) {
                _entry_flags[ii] |= OME_PRINT_VALUE;
            }
        }
#endif
    }

    // skip labels, strings, and dynamic read-only entries
    while (row_is_skippable(_page_top + _current_menu_context.cursor_row)) {
        ++_current_menu_context.cursor_row;
    }

    // position the menu in the bottom half of the screen
    const uint8_t topRow = _small_screen ? 1 : static_cast<uint8_t>((ctx.display_port.get_row_count() - _page_max_row)/2);
    auto spaceLeft = static_cast<int32_t>(ctx.display_port.tx_bytes_free());

    if (_cursor_row != _current_menu_context.cursor_row) {
        // cursor position has changed, so:
        // clear the old cursor
        if (_cursor_row != CURSOR_ROW_NOT_SET) {
            const uint8_t row = topRow + static_cast<uint8_t>(_cursor_row * _lines_per_menu_item);
            spaceLeft -= static_cast<int32_t>(ctx.display_port.write_string(_left_menu_column, row, " "));
        }
        // and draw the new one
        _cursor_row = _current_menu_context.cursor_row;
    }
    const uint8_t row = topRow + static_cast<uint8_t>(_cursor_row * _lines_per_menu_item);
    spaceLeft -= static_cast<int32_t>(ctx.display_port.write_string(_left_menu_column, row, ">"));
    if (_current_menu_context.menu->on_display_update) {
        if (_current_menu_context.menu->on_display_update(*this, ctx, _page_top + _current_menu_context.cursor_row) == MENU_BACK) {
            menu_back(ctx, nullptr);
            return;
        }
    }

    // Display the menu entries
    uint8_t ii = 0;
    for (const OSD_Entry* entry = _page_top; entry <= _page_top + _page_max_row; ++ii, ++entry) {
        const uint8_t entryRow = topRow + static_cast<uint8_t>(ii * _lines_per_menu_item);
        // display the current item indicator
        //uint16_t& entry_flags = _entry_flags[ii];
        // Highlight values overridden by sliders
        if (row_slider_override(entry->flags)) { // cppcheck-suppress knownConditionTrueFalse
            ctx.display_port.write_char(_left_menu_column - 1, entryRow, 'S');
        }
        // Print values
        // XXX Polled values at latter positions in the list may not be
        // XXX printed if not enough room in the middle of the list.
        if ((_entry_flags[ii] & OME_PRINT_VALUE) || (_entry_flags[ii] & OME_SCROLLING_TICKER)) {
            //const bool selectedRow = (ii == _current_menu_context.cursor_row);
            spaceLeft -= static_cast<int32_t>(draw_menuEntry(ctx, entry, entryRow, _entry_flags[ii], _runtimeTableTicker[ii]));
            enum { CHARACTERS_PER_LINE };
            if (spaceLeft < CHARACTERS_PER_LINE) {
                return;
            }
        }
    }

    // Draw the up/down page indicators if the display has space.
    // Only draw the symbols when necessary after the screen has been cleared. Otherwise they're static.
    if (displayWasCleared && _left_menu_column > 0) { // make sure there's room to draw the symbol
        if (_current_menu_context.page > 0) {
            ctx.display_port.write_char(_left_menu_column - 1, topRow, ctx.display_port.get_small_arrow_up());
        }
        if (_current_menu_context.page < _page_count - 1) {
            ctx.display_port.write_char(_left_menu_column - 1, topRow + _page_max_row, ctx.display_port.get_small_arrow_down());
        }
    }
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

uint16_t CMSX::handle_key(cms_context_t& ctx, key_e key) // NOLINT(readability-function-cognitive-complexity)
{
    if (!_current_menu_context.menu) {
        return BUTTON_TIME_MS;
    }
    if (key == KEY_MENU) {
        menu_open(ctx);
        return BUTTON_PAUSE_MS;
    }
    if (key == KEY_ESC) {
        if (_elementEditing) {
            _elementEditing = false;
        } else {
            menu_back(ctx, nullptr);
        }
        return BUTTON_PAUSE_MS;
    }
    if (key == KEY_SAVE_MENU && !_save_menu_inhibited) {
        _elementEditing = false;
        menu_change(*this, ctx, get_save_exit_menu());
        return BUTTON_PAUSE_MS;
    }
    if (!_elementEditing) {
        if (key == KEY_DOWN) {
            if (_current_menu_context.cursor_row < _page_max_row) {
                ++_current_menu_context.cursor_row;
            } else {
                page_next();
                ctx.display_port.clear_screen(DISPLAY_CLEAR_WAIT);
                _current_menu_context.cursor_row = 0;    // Goto top in any case
            }
            return BUTTON_TIME_MS;
        }
        if (key == KEY_UP) {
            auto cursor_row = static_cast<int8_t>(_current_menu_context.cursor_row);
            --cursor_row;
            // Skip non-title labels, strings and dynamic read-only entries
            while ((row_is_skippable(_page_top + _current_menu_context.cursor_row)) && cursor_row > 0) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
                --cursor_row;
            }
            if (cursor_row == -1 || ((_page_top + cursor_row)->flags & OME_TYPE_MASK) == OME_LABEL) { //NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
                // Goto previous page
                page_previous();
                ctx.display_port.clear_screen(DISPLAY_CLEAR_WAIT);
                _current_menu_context.cursor_row = _page_max_row;
            } else {
                _current_menu_context.cursor_row = static_cast<uint8_t>(cursor_row);
            }
            return BUTTON_TIME_MS;
        }
    }
    const OSD_Entry* entry = _page_top + _current_menu_context.cursor_row; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return handle_key(ctx, key, entry, _entry_flags[_current_menu_context.cursor_row]);
}

uint16_t CMSX::handle_key(cms_context_t& ctx, key_e key, const OSD_Entry* entry, uint16_t& entry_flags) // NOLINT(readability-function-cognitive-complexity)
{
//NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    uint16_t ret = BUTTON_TIME_MS;

    switch (entry->flags & OME_TYPE_MASK) {
    case OME_LABEL:
        break;
    case OME_SUBMENU:
        if (key == KEY_RIGHT) {
            menu_change(*this, ctx, reinterpret_cast<const menu_t*>(entry->data));
            ret = BUTTON_PAUSE_MS;
        }
        break;
    case OME_FUNCTION_CALL:
        if (entry->fnPtr && key == KEY_RIGHT) {
            if (entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data)) == MENU_BACK) {
                menu_back(ctx, nullptr);
            }
            if ((entry->flags & OME_REBOOT_REQUIRED)) {
                set_reboot_required();
            }
            ret = BUTTON_PAUSE_MS;
        }
        break;
    case OME_EXIT:
        if (entry->fnPtr && key == KEY_RIGHT) {
            entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data));
            ret = BUTTON_PAUSE_MS;
        }
        break;
    case OME_BACK:
        menu_back(ctx, nullptr);
        ret = BUTTON_PAUSE_MS;
        _elementEditing = false;
        break;
    case OME_END:
        break;
#if defined(USE_OSD)
    case OME_VISIBLE:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const osd_uint16_t*>(entry->data);
            const uint16_t previousValue = *ptr->val;
            if ((key == KEY_RIGHT) && (!_elementEditing)) {
                _elementEditing = true;
                _osd_profile_cursor = 0;
            } else if (_elementEditing) {
                if (key == KEY_RIGHT) {
                    if (_osd_profile_cursor < OSD_Elements::PROFILE_COUNT) {
                        ++_osd_profile_cursor;
                    }
                }
                if (key == KEY_LEFT) {
                    if (_osd_profile_cursor > 0) {
                        --_osd_profile_cursor;
                    }
                }
                if (key == KEY_UP) {
                    set_flag(*ptr->val, OSD_Elements::profile_flag(_osd_profile_cursor));
                }
                if (key == KEY_DOWN) {
                    clear_flag(*ptr->val, OSD_Elements::profile_flag(_osd_profile_cursor));
                }
            }
            set_flag(entry_flags, OME_PRINT_VALUE);

            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                set_reboot_required();
            }
        }
        break;
#endif
    case OME_STRING:
        break;
    case OME_TABLE:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const osd_table_t*>(entry->data);
            const uint8_t previousValue = *ptr->val;
            if (key == KEY_RIGHT) {
                if (*ptr->val < ptr->max) {
                    *ptr->val += 1;
                }
            } else {
                if (*ptr->val > 0) {
                    *ptr->val -= 1;
                }
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data));
            }
            set_flag(entry_flags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                set_reboot_required();
            }
        }
        break;
    case OME_BOOL:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const osd_bool_t*>(entry->data);
            const bool previousValue = *ptr->val;
            *ptr->val = (key == KEY_RIGHT) ? true : false;
            set_flag(entry_flags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                set_reboot_required();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_INT8:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const osd_int8_t*>(entry->data);
            const int8_t previousValue = *ptr->val;
            if (key == KEY_RIGHT) {
                if (*ptr->val < ptr->max) {
                    *ptr->val = static_cast<int8_t>(*ptr->val + ptr->step);
                }
            } else {
                if (*ptr->val > ptr->min) {
                    *ptr->val = static_cast<int8_t>(*ptr->val - ptr->step);
                }
            }
            set_flag(entry_flags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                set_reboot_required();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_UINT8_FIXED:
        [[fallthrough]];
    case OME_UINT8:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const osd_uint8_t*>(entry->data);
            const uint16_t previousValue = *ptr->val;
            if (key == KEY_RIGHT) {
                if (*ptr->val < ptr->max) {
                    *ptr->val += ptr->step;
                }
            } else {
                if (*ptr->val > ptr->min) {
                    *ptr->val -= ptr->step;
                }
            }
            set_flag(entry_flags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                set_reboot_required();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_UINT16_FIXED:
        [[fallthrough]];
    case OME_UINT16:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const osd_uint16_t*>(entry->data);
            const uint16_t previousValue = *ptr->val;
            if (key == KEY_RIGHT) {
                if (*ptr->val < ptr->max) {
                    *ptr->val += ptr->step;
                }
            } else {
                if (*ptr->val > ptr->min) {
                    *ptr->val -= ptr->step;
                }
            }
            set_flag(entry_flags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                set_reboot_required();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_INT16:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const osd_int16_t*>(entry->data);
            const int16_t previousValue = *ptr->val;
            if (key == KEY_RIGHT) {
                if (*ptr->val < ptr->max) {
                    *ptr->val = static_cast<int16_t>(*ptr->val + ptr->step);
                }
            } else {
                if (*ptr->val > ptr->min) {
                    *ptr->val = static_cast<int16_t>(*ptr->val - ptr->step);
                }
            }
            set_flag(entry_flags, OME_PRINT_VALUE);

            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                set_reboot_required();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_UINT32:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const osd_uint32_t*>(entry->data);
            const uint32_t previousValue = *ptr->val;
            if (key == KEY_RIGHT) {
                if (*ptr->val < ptr->max) {
                    *ptr->val += ptr->step;
                }
            } else {
                if (*ptr->val > ptr->min) {
                    *ptr->val -= ptr->step;
                }
            }
            set_flag(entry_flags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                set_reboot_required();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_INT32:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const osd_int32_t*>(entry->data);
            const int32_t previousValue = *ptr->val;
            if (key == KEY_RIGHT) {
                if (*ptr->val < ptr->max) {
                    *ptr->val += ptr->step;
                }
            } else {
                if (*ptr->val > ptr->min) {
                    *ptr->val -= ptr->step;
                }
            }
            set_flag(entry_flags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                set_reboot_required();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, ctx, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_MENU:
        // Shouldn't happen
        break;
    }
    return ret;
//NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
}

void CMSX::menu_open(cms_context_t& ctx)
{
    const CMSX::menu_t* startMenu = _current_menu_context.menu;
    if (_in_menu) {
        // Switch display
        //DisplayPortBase* nextDisplayPort = _cms.display_portSelectNext();
        //if (nextDisplayPort == &display_port) {
        //    return;
        //}
        // DisplayPort has been changed.
        //_current_menu_context.cursor_row += static_cast<uint8_t>(_current_menu_context.page * _max_menu_items);
        ctx.display_port.set_background_type(DisplayPortBase::BACKGROUND_TRANSPARENT); // reset previous display_port to transparent
        //display_port.release();
        //_cms.setDisplayPort(nextDisplayPort);
    } else {
        //_display_port = cmsDisplayPortSelectCurrent();
        //if (!_display_port) {
        //    return;
        //}
        _in_menu = true;
        startMenu = &_menu_main;
        _current_menu_context = { nullptr, 0, 0 };
        menu_stack_reset();
        set_arming_disabled(ctx);
        _cursor_row = CURSOR_ROW_NOT_SET;
        ctx.display_port.clear_screen(DISPLAY_CLEAR_WAIT);
        ctx.display_port.grab();
        ctx.display_port.layer_select(DisplayPortBase::LAYER_FOREGROUND);
    }
    //!!TODO: this should not have a dependency on the OSD
#if defined(USE_OSD)
    //!!_resume_refresh_at_us = 0;
#endif

    const uint8_t column_count = ctx.display_port.get_column_count();
    if (column_count < NORMAL_SCREEN_MIN_COLS) {
        _small_screen       = true;
        _right_aligned      = true;
        _lines_per_menu_item  = 2;
        _left_menu_column    = 0;
        _right_menu_column   = column_count;
        _max_menu_items      = ctx.display_port.get_row_count() / _lines_per_menu_item;
    } else {
        _small_screen       = false;
        _right_aligned      = true;
        _lines_per_menu_item  = 1;
        if (column_count <= NORMAL_SCREEN_MAX_COLS) {
            _left_menu_column    = 1;
            _right_menu_column   = _right_aligned ? column_count - 2 : column_count - MENU_DRAW_BUFFER_LEN; // cppcheck-suppress knownConditionTrueFalse
        } else {
            _left_menu_column    = (column_count / 2) - 13;
            _right_menu_column   = _right_aligned ? (column_count / 2) + 13 : column_count - MENU_DRAW_BUFFER_LEN;
        }
        _max_menu_items = ctx.display_port.get_row_count() - 2;
    }

    if (ctx.display_port.get_use_full_screen()) {
        _left_menu_column = 0;
        _right_menu_column   = column_count;
        _max_menu_items      = ctx.display_port.get_row_count();
    }
    CMSX::menu_change(ctx, startMenu);
}

const void* CMSX::menu_change(cms_context_t& ctx, const menu_t* menu)
{
    if (!menu) {
        return nullptr;
    }
    if (menu == _current_menu_context.menu) {
        const uint8_t cursorAbs = _current_menu_context.cursor_row;
       _current_menu_context.cursor_row = cursorAbs %_max_menu_items;
       page_select(cursorAbs /_max_menu_items);
        ctx.display_port.clear_screen(DISPLAY_CLEAR_WAIT);
    } else {
        if (_current_menu_context.menu && menu != &_menu_main) {
            // If we are opening the initial top-level menu, then _current_menu_context.menu will be nullptr and there is nothing to do.
            // Otherwise stack the current menu before moving to the selected menu.
            if (menu_stack_push() == MENU_STACK_NO_ROOM_TO_PUSH) {
                return nullptr;
            }
        }
       _current_menu_context.menu = menu;
       _current_menu_context.cursor_row = 0;
        if (menu->on_enter) {
            if (menu->on_enter(*this, ctx) == MENU_BACK) {
                return menu_back(ctx, menu);
            }
        }
        page_select(0);
        ctx.display_port.clear_screen(DISPLAY_CLEAR_WAIT);
    }
    return nullptr;
}

const void* CMSX::menu_back(cms_context_t& ctx, const menu_t* menu)
{
    (void)menu;

    if (_current_menu_context.menu->on_exit) {
        if (_current_menu_context.menu->on_exit(*this, ctx,_page_top +_current_menu_context.cursor_row) == MENU_BACK) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            return MENU_BACK;
        }
    }
   _save_menu_inhibited = false;
    if (menu_stack_pop() == MENU_STACK_NOTHING_TO_POP) {
        return nullptr;
    }
    page_select(_current_menu_context.page);
    ctx.display_port.clear_screen(DISPLAY_CLEAR_WAIT);

    return nullptr;
}

const void* CMSX::menu_exit(cms_context_t& ctx, const menu_t* menu)
{
    if (menu == MENU_EXIT_SAVE || menu == MENU_EXIT_SAVE_REBOOT || menu == MENU_POPUP_SAVE || menu == MENU_POPUP_SAVE_REBOOT) {
        traverse_global_exit(&_menu_main);
        if (_current_menu_context.menu->on_exit) {
            _current_menu_context.menu->on_exit(*this, ctx, nullptr); // Forced exit
        }
        if ((menu == MENU_POPUP_SAVE) || (menu == MENU_POPUP_SAVE_REBOOT)) {
            // traverse through the menu stack and call all their on_exit functions
            for (int ii = _menu_stack_index - 1; ii >= 0; --ii) {
                if (_menu_stack[static_cast<size_t>(ii)].menu->on_exit) {
                   _menu_stack[static_cast<size_t>(ii)].menu->on_exit(*this, ctx, nullptr);
                }
            }
        }
        //save_config_and_notify(ctx);
    }

    ctx.display_port.set_background_type(DisplayPortBase::BACKGROUND_TRANSPARENT);
   _in_menu = false;
    ctx.display_port.release();
   _current_menu_context.menu = nullptr;

    if ((menu == MENU_EXIT_SAVE_REBOOT) || (menu == MENU_POPUP_SAVE_REBOOT) || (menu == MENU_POPUP_EXIT_REBOOT)) {
        _cursor_row = CURSOR_ROW_NOT_SET;
        ctx.display_port.clear_screen(DISPLAY_CLEAR_WAIT);
        ctx.display_port.write_string(5, 3, "REBOOTING...");
        ctx.display_port.redraw();
#if false
        stopMotors();
        motorShutdown();
        delay(200);

        systemReset();
#endif
    }

    clear_arming_disabled(ctx);

    return nullptr;
}

void CMSX::set_arming_disabled(cms_context_t& ctx)
{
    ctx.cockpit.set_arming_disabled_flag(ARMING_DISABLED_CMS_MENU);
}

void CMSX::clear_arming_disabled(cms_context_t& ctx)
{
    ctx.cockpit.clear_arming_disabled_flag(ARMING_DISABLED_CMS_MENU);
}


void CMSX::page_select(uint8_t newpage)
{
    const OSD_Entry* menuEntry = _current_menu_context.menu->entries;
    while ((menuEntry->flags & OME_TYPE_MASK) != OME_END) {
        ++menuEntry; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    _page_count = static_cast<uint8_t>(((menuEntry - _current_menu_context.menu->entries - 1) / _max_menu_items) + 1);

    _current_menu_context.page = static_cast<uint8_t>((newpage + _page_count) % _page_count);
    _page_top = &_current_menu_context.menu->entries[_current_menu_context.page * _max_menu_items]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,bugprone-implicit-widening-of-multiplication-result)

    uint8_t ii = 0;
    for (const OSD_Entry* entry = _page_top; (entry->flags & OME_TYPE_MASK) != OME_END; ++entry) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        _entry_flags[ii] = entry->flags;
        ++ii;
    }
    _page_max_row = ii;
    if (_page_max_row > _max_menu_items) {
        _page_max_row = _max_menu_items;
    }
    if (_page_max_row > MAX_ROWS) {
        _page_max_row = MAX_ROWS;
    }
    --_page_max_row;
    _cursor_row = CURSOR_ROW_NOT_SET;
}

void CMSX::page_next()
{
    page_select(_current_menu_context.page + 1);
}

void CMSX::page_previous()
{
    page_select(_current_menu_context.page - 1);
}

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast,hicpp-signed-bitwise,misc-no-recursion)
void CMSX::traverse_global_exit(const CMSX::menu_t* menu)
{
    for (const CMSX::OSD_Entry* entry = menu->entries; (entry->flags & OME_TYPE_MASK) != OME_END; ++entry) {
        if ((entry->flags & OME_TYPE_MASK) == OME_SUBMENU) {
            traverse_global_exit(reinterpret_cast<const CMSX::menu_t*>(entry->data));
        }
    }
}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast,hicpp-signed-bitwise,misc-no-recursion)

void CMSX::menu_stack_reset()
{
    _menu_stack_index = 0;
}

CMSX::menu_stack_e CMSX::menu_stack_push() // NOLINT(readability-make-member-function-const)
{
    if (_menu_stack_index >= MAX_MENU_STACK_DEPTH - 1) {
         return MENU_STACK_NO_ROOM_TO_PUSH;
    }
    _menu_stack[_menu_stack_index++] = _current_menu_context;
    return MENU_STACK_OK;
}

CMSX::menu_stack_e  CMSX::menu_stack_pop() // NOLINT(readability-make-member-function-const)
{
    if (_menu_stack_index == 0) {
        return MENU_STACK_NOTHING_TO_POP;
    }
    _current_menu_context = _menu_stack[--_menu_stack_index];
    return MENU_STACK_OK;
}
