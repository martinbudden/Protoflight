#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "DisplayPortBase.h"
#include "OSD_Symbols.h"
#include <cstring>


//NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
const CMSX::menu_t* CMSX::MENU_NULL_PTR         = nullptr;
const CMSX::menu_t* CMSX::MENU_EXIT              = CMSX::MENU_NULL_PTR + 1;
const CMSX::menu_t* CMSX::MENU_EXIT_SAVE         = CMSX::MENU_NULL_PTR + 2;
const CMSX::menu_t* CMSX::MENU_EXIT_SAVE_REBOOT  = CMSX::MENU_NULL_PTR + 3;
const CMSX::menu_t* CMSX::MENU_POPUP_SAVE        = CMSX::MENU_NULL_PTR + 4;
const CMSX::menu_t* CMSX::MENU_POPUP_SAVE_REBOOT = CMSX::MENU_NULL_PTR + 5;
const CMSX::menu_t* CMSX::MENU_POPUP_EXIT_REBOOT = CMSX::MENU_NULL_PTR + 6;
const CMSX::menu_t* CMSX::MENU_CHAIN_BACK       = CMSX::MENU_NULL_PTR + 7;
//NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)


CMSX::CMSX(CMS& cms) :
    _cms(cms)
{
}

void CMSX::setRebootRequired()
{
}

void CMSX::setInMenu(bool inMenu)
{
    _inMenu = inMenu;
}

bool CMSX::isInMenu() const
{
    return _inMenu;
}

uint8_t CMSX::cursorAbsolute() const
{ 
    return _currentCtx.cursorRow + static_cast<uint8_t>(_currentCtx.page * _maxMenuItems);
}

// Check if overridden by slider, used by simplified PID tuning
bool CMSX::rowSliderOverride(const uint16_t flags)
{
    (void)flags;
    return false;
}

bool CMSX::rowIsSkippable(const OSD_Entry* row)
{
    const uint16_t type = row->flags & OME_MASK;

    if (type == OME_Label) {
        return true;
    }
    if (type == OME_String) {
        return true;
    }
    if ((row->flags == OME_DYNAMIC) || rowSliderOverride(row->flags)) { // cppcheck-suppress knownConditionTrueFalse
        if (type == OME_UINT8  || type == OME_INT8 || type == OME_UINT16  || type == OME_INT16) {
            return true;
        }
    }
    return false;
}

bool CMSX::elementVisible(uint16_t value) const
{
    static constexpr uint16_t PROFILE_BITS_POS = 12;
    static constexpr uint16_t PROFILE_MASK = 0b0011'0000'0000'0000U;
    return ((value & PROFILE_MASK) >> PROFILE_BITS_POS) & (1U << _profile); // NOLINT(hicpp-signed-bitwise)
}

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
void CMSX::drawMenu(DisplayPortBase& displayPort, uint32_t currentTimeUs) // NOLINT(readability-function-cognitive-complexity)
{
    (void)currentTimeUs;
    if (!_pageTop || !_inMenu) {
        return;
    }
    const bool displayWasCleared = displayPort.isCleared();
    displayPort.setCleared(false);

    bool drawPolled = false;
    if (currentTimeUs > _lastPolledUs + CMS_POLL_INTERVAL_US) {
        drawPolled = true;
        _lastPolledUs = currentTimeUs;
    }

    uint32_t room = displayPort.txBytesFree();

    uint8_t ii = 0;
    if (displayWasCleared) {
        for (const OSD_Entry* entry = _pageTop; entry <= _pageTop + _pageMaxRow; ++entry, ++ii) {
            _runtimeEntryFlags[ii] |= OME_PRINT_LABEL | OME_PRINT_VALUE;
        }
    } else if (drawPolled) {
        for (const OSD_Entry* entry = _pageTop; entry <= _pageTop + _pageMaxRow; ++entry, ++ii) {
            if (entry->flags & OME_DYNAMIC) {
                _runtimeEntryFlags[ii] |= OME_PRINT_VALUE;
            }
        }
    }
    while (rowIsSkippable(_pageTop + _currentCtx.cursorRow)) { // skip labels, strings and dynamic read-only entries
        ++_currentCtx.cursorRow;
    }
    const uint8_t top = _smallScreen ? 1 : static_cast<uint8_t>((displayPort.getRowCount() - _pageMaxRow)/2);
    const uint8_t y = top + static_cast<uint8_t>(displayPort.getCursorRow() * _linesPerMenuItem);
    if (_currentCtx.cursorRow != displayPort.getCursorRow()) {
        room -= displayPort.writeString(_leftMenuColumn, y, DisplayPortBase::SEVERITY_NORMAL, " ");
    }
    if (room < 30) {
        return;
    }
    if (displayPort.getCursorRow() != _currentCtx.cursorRow) {
        room -= displayPort.writeString(_leftMenuColumn, y, DisplayPortBase::SEVERITY_NORMAL, ">");
        displayPort.setCursorRow(_currentCtx.cursorRow);
    }
    if (room < 30) {
        return;
    }
    if (_currentCtx.menu->onDisplayUpdate) {
        const void* result = _currentCtx.menu->onDisplayUpdate(*this, displayPort, _pageTop + _currentCtx.cursorRow);
        if (result == MENU_CHAIN_BACK) {
            menuBack(*this, displayPort, nullptr);
            return;
        }
    }
    // Print text labels
    ii = 0;
    for (const OSD_Entry* entry = _pageTop; (entry <= _pageTop + _pageMaxRow); ++ii, ++entry) {
        const uint8_t yVal = top + static_cast<uint8_t>(ii * _linesPerMenuItem);
        if (_runtimeEntryFlags[ii] & OME_PRINT_LABEL) {
            uint8_t coloff = _leftMenuColumn;
            coloff += ((entry->flags & OME_MASK) == OME_Label) ? 0 : 1;
            room -= displayPort.writeString(coloff, yVal, DisplayPortBase::SEVERITY_NORMAL, entry->text);
            clearFlag(_runtimeEntryFlags[ii], OME_PRINT_LABEL);
            if (room < 30) {
                return;
            }
        }
        // Highlight values overridden by sliders
        if (rowSliderOverride(entry->flags)) { // cppcheck-suppress knownConditionTrueFalse
            displayPort.writeChar(_leftMenuColumn - 1, yVal, DisplayPortBase::SEVERITY_NORMAL, 'S');
        }
        // Print values
        // XXX Polled values at latter positions in the list may not be
        // XXX printed if not enough room in the middle of the list.
        if ((_runtimeEntryFlags[ii] & OME_PRINT_VALUE) || (_runtimeEntryFlags[ii] & OME_SCROLLING_TICKER)) {
            const bool selectedRow = (ii == _currentCtx.cursorRow);
            room -= drawMenuEntry(displayPort, entry, yVal, selectedRow, ii);
            if (room < 30) {
                return;
            }
        }
    }
    // Draw the up/down page indicators if the display has space.
    // Only draw the symbols when necessary after the screen has been cleared. Otherwise they're static.
    // If the device supports OSD symbols then use the up/down arrows. Otherwise assume it's a
    // simple text device and use the '^' (carat) and 'V' for arrow approximations.
    if (displayWasCleared && _leftMenuColumn > 0) {      // make sure there's room to draw the symbol
        if (_currentCtx.page > 0) {
            const uint8_t symbol = displayPort.supportsOsdSymbols() ? SYM_ARROW_SMALL_UP : '^';
            displayPort.writeChar(_leftMenuColumn - 1, top, DisplayPortBase::SEVERITY_NORMAL, symbol);
        }
        if (_currentCtx.page < _pageCount - 1) {
            static_assert(SYM_ARROW_SMALL_DOWN == 'v');
            displayPort.writeChar(_leftMenuColumn - 1, top + _pageMaxRow, DisplayPortBase::SEVERITY_NORMAL, SYM_ARROW_SMALL_DOWN);
        }
    }
}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)

// Pad buffer to the right, i.e. align left
void CMSX::padRight(char *buf, uint8_t size)
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
void CMSX::padLeft(char *buf, uint8_t size)
{
    auto len = static_cast<uint8_t>(strlen(buf));

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

void CMSX::padToSize(char* buf, uint8_t size) const
{
#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
    cmsPadLeft(buf, size);
#else
    _smallScreen ? padLeft(buf, size) : padRight(buf, size);
#endif
}

uint32_t CMSX::drawMenuItemValue(DisplayPortBase& displayPort, char* buf, uint8_t row, uint8_t maxSize) const
{
    padToSize(buf, maxSize);
#if defined(CMS_OSD_RIGHT_ALIGNED_VALUES)
    const uint8_t column = _rightMenuColumn - maxSize;
#else
    const uint8_t column = _smallScreen ? _rightMenuColumn - maxSize : _rightMenuColumn;
#endif
    return displayPort.writeString(column, row, DisplayPortBase::SEVERITY_NORMAL, buf);
}

static void ui2a(unsigned int num, char* bf)
{
    const unsigned int base = 10;

    unsigned int d = 1;
    while (num / d >= base) {
        d *= base;
    }
    while (d != 0) {
        const unsigned int dgt = num / d;
        *bf++ = static_cast<char>(dgt + '0' - 10); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        // Next digit
        num %= d;
        d /= base;
    }
    *bf = 0;
}

static void i2a(int num, char* bf)
{
    if (num < 0) {
        num = -num;
        *bf++ = '-'; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    ui2a(static_cast<unsigned int>(num), bf);
}

uint32_t CMSX::drawMenuEntry(DisplayPortBase& displayPort, const OSD_Entry *entry, uint8_t row, bool selectedRow, uint8_t index) // NOLINT(readability-function-cognitive-complexity)
{
    (void)selectedRow;

    uint32_t count = 0;

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-pointer-arithmetic)
    switch (entry->flags & OME_MASK) {
    case OME_Label:
        if ((_runtimeEntryFlags[index] & OME_PRINT_VALUE) && entry->data) {
            // A label with optional string, immediately following text
            count = displayPort.writeString(static_cast<uint8_t>(_leftMenuColumn + strlen(entry->text) + 1), row, DisplayPortBase::SEVERITY_NORMAL, reinterpret_cast<const uint8_t*>(entry->data));
            clearFlag(_runtimeEntryFlags[index], OME_PRINT_VALUE);
        }
        break;
    case OME_Submenu:
        [[fallthrough]];
    case OME_FunctionCall:
        if ((_runtimeEntryFlags[index] & OME_PRINT_VALUE)) {
            _menuDrawBuf[0] = 0;

            if ((entry->flags & OME_MASK) == OME_Submenu && entry->fnPtr && _runtimeEntryFlags[index] & OME_OPTSTRING) {

                // Special case of sub menu entry with optional value display.

                const char *str = reinterpret_cast<const char*>(entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data)));
                strncpy(&_menuDrawBuf[0], str, MENU_DRAW_BUFFER_LEN);
            } else if ((entry->flags & OME_MASK) == OME_FunctionCall && entry->data) {
                strncpy(&_menuDrawBuf[0], reinterpret_cast<const char*>(entry->data), MENU_DRAW_BUFFER_LEN);
            }
            strncat(&_menuDrawBuf[0], ">", MENU_DRAW_BUFFER_LEN);

            row = _smallScreen ? row - 1 : row;
            count = drawMenuItemValue(displayPort, &_menuDrawBuf[0], row, static_cast<uint8_t>(strlen(&_menuDrawBuf[0])));
            clearFlag(_runtimeEntryFlags[index], OME_PRINT_VALUE);
        }
        break;
    case OME_INT8:
        if ((_runtimeEntryFlags[index] & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_INT8_t*>(entry->data);
            i2a(*ptr->val, &_menuDrawBuf[0]);
            count = drawMenuItemValue(displayPort, &_menuDrawBuf[0], row, NUMBER_FIELD_LEN);
            clearFlag(_runtimeEntryFlags[index], OME_PRINT_VALUE);
        }
        break;
    case OME_UINT8:
        if ((_runtimeEntryFlags[index] & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT8_t*>(entry->data);
            i2a(*ptr->val, &_menuDrawBuf[0]);
            count = drawMenuItemValue(displayPort, &_menuDrawBuf[0], row, NUMBER_FIELD_LEN);
            clearFlag(_runtimeEntryFlags[index], OME_PRINT_VALUE);
        }
        break;
    case OME_INT16:
        if ((_runtimeEntryFlags[index] & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_INT16_t*>(entry->data);
            i2a(*ptr->val, &_menuDrawBuf[0]);
            count = drawMenuItemValue(displayPort, &_menuDrawBuf[0], row, NUMBER_FIELD_LEN);
            clearFlag(_runtimeEntryFlags[index], OME_PRINT_VALUE);
        }
        break;
    case OME_UINT16:
        if ((_runtimeEntryFlags[index] & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT16_t*>(entry->data);
            ui2a(*ptr->val, &_menuDrawBuf[0]);
            count = drawMenuItemValue(displayPort, &_menuDrawBuf[0], row, NUMBER_FIELD_LEN);
            clearFlag(_runtimeEntryFlags[index], OME_PRINT_VALUE);
        }
        break;
    case OME_INT32:
        if ((_runtimeEntryFlags[index] & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_INT32_t*>(entry->data);
            i2a(*ptr->val, &_menuDrawBuf[0]);
            count = drawMenuItemValue(displayPort, &_menuDrawBuf[0], row, NUMBER_FIELD_LEN);
            clearFlag(_runtimeEntryFlags[index], OME_PRINT_VALUE);
        }
        break;
    case OME_UINT32:
        if ((_runtimeEntryFlags[index] & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT32_t*>(entry->data);
            ui2a(*ptr->val, &_menuDrawBuf[0]);
            count = drawMenuItemValue(displayPort, &_menuDrawBuf[0], row, NUMBER_FIELD_LEN);
            clearFlag(_runtimeEntryFlags[index], OME_PRINT_VALUE);
        }
        break;
    case OME_String:
        if ((_runtimeEntryFlags[index] & OME_PRINT_VALUE) && entry->data) {
            strncpy(reinterpret_cast<char*>(&_menuDrawBuf[0]), static_cast<const char*>(entry->data), MENU_DRAW_BUFFER_LEN);
            count = drawMenuItemValue(displayPort, &_menuDrawBuf[0], row, MENU_DRAW_BUFFER_LEN);
            clearFlag(_runtimeEntryFlags[index], OME_PRINT_VALUE);
        }
        break;
    case OME_TABLE:
        if ((_runtimeEntryFlags[index] & OME_PRINT_VALUE) || (_runtimeEntryFlags[index] & OME_SCROLLING_TICKER)) {
            const auto* ptr = reinterpret_cast<const OSD_TABLE_t*>(entry->data);
            const size_t labelLength = strlen(entry->text) + 1; // account for the space between label and display data
            const char* str = static_cast<const char *>(ptr->names[*ptr->val]);   // lookup table display text
            const size_t displayLength = strlen(str);
            // Calculate the available space to display the lookup table entry based on the
            // screen size and the length of the label. Always display at least MENU_DRAW_BUFFER_LEN
            // characters to prevent really long labels from overriding the data display.
            const size_t availableSpace = std::max(static_cast<size_t>(MENU_DRAW_BUFFER_LEN), _rightMenuColumn - labelLength - _leftMenuColumn - 1);

            bool drawText = false;
            table_ticker_t& ticker = _runtimeTableTicker[index];
            if (_runtimeEntryFlags[index] & OME_PRINT_VALUE) {
                drawText = true;
                _runtimeTableTicker[index].state = 0;
                ticker.loopCounter = 0;
                if (displayLength > availableSpace) {  // table entry text is longer than the available space so start the ticker
                    setFlag(_runtimeEntryFlags[index], OME_SCROLLING_TICKER);
                } else {
                    clearFlag(_runtimeEntryFlags[index], OME_SCROLLING_TICKER);
                }
            } else {
                ++ticker.loopCounter;
                const uint8_t loopLimit = (ticker.state == 0 || ticker.state == (displayLength - availableSpace)) ? LOOKUP_TABLE_TICKER_START_CYCLES : LOOKUP_TABLE_TICKER_SCROLL_CYCLES;
                if (ticker.loopCounter >= loopLimit) {
                    ticker.loopCounter = 0;
                    drawText = true;
                    ticker.state++;
                    if (ticker.state > (displayLength - availableSpace)) {
                        ticker.state = 0;
                    }
                }
            }
            if (drawText) {
                strncpy(&_menuTableBuf[0], static_cast<const char *>(str + ticker.state), MENU_TABLE_BUFFER_LEN);
                count = drawMenuItemValue(displayPort, &_menuTableBuf[0], row, static_cast<uint8_t>(availableSpace));
            }
            clearFlag(_runtimeEntryFlags[index], OME_PRINT_VALUE);
        }
        break;

    }
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return count;
}


void CMSX::addMenuEntry(OSD_Entry& menuEntry, const char* text, uint16_t flags, CMSX::entryFnPtr fnPtr, void* data)
{
    menuEntry.text = text;
    menuEntry.flags = flags;
    menuEntry.fnPtr = fnPtr;
    menuEntry.data = data;
}

uint16_t CMSX::handleKey(DisplayPortBase& displayPort, key_e key) // NOLINT(readability-function-cognitive-complexity)
{
    uint16_t ret = BUTTON_TIME_MS;
    if (!_currentCtx.menu) {
        return ret;
    }

    if (key == KEY_MENU) {
        menuOpen( displayPort);
        return BUTTON_PAUSE_MS;
    }
    if (key == KEY_ESC) {
        if (_elementEditing) {
            _elementEditing = false;
        } else {
            menuBack(displayPort);
        }
        return BUTTON_PAUSE_MS;
    }
    if (key == KEY_SAVE_MENU && !_saveMenuInhibited) {
        _elementEditing = false;
        menuChange(*this, displayPort, getSaveExitMenu());
        return BUTTON_PAUSE_MS;
    }

    if ((key == KEY_DOWN) && (!_elementEditing)) {
        if (_currentCtx.cursorRow < _pageMaxRow) {
            ++_currentCtx.cursorRow;
        } else {
            pageNext(displayPort);
            _currentCtx.cursorRow = 0;    // Goto top in any case
        }
    }

    if ((key == KEY_UP) && (!_elementEditing)) {
        auto cursorRow = static_cast<int8_t>(_currentCtx.cursorRow);
        --cursorRow;
        // Skip non-title labels, strings and dynamic read-only entries
        while ((rowIsSkippable(_pageTop + _currentCtx.cursorRow)) && cursorRow > 0) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            --cursorRow;
        }
        if (cursorRow == -1 || ((_pageTop + cursorRow)->flags & OME_MASK) == OME_Label) { //NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            // Goto previous page
            pagePrevious(displayPort);
            _currentCtx.cursorRow = _pageMaxRow;
        } else {
            _currentCtx.cursorRow = static_cast<uint8_t>(cursorRow);
        }
    }

    if ((key == KEY_DOWN || key == KEY_UP) && (!_elementEditing)) {
        return ret;
    }

//NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    const OSD_Entry* entry = _pageTop + _currentCtx.cursorRow; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    switch (entry->flags & OME_MASK) {
    case OME_Label:
        break;
    case OME_Back:
        menuBack(displayPort);
        ret = BUTTON_PAUSE_MS;
        _elementEditing = false;
        break;
    case OME_Submenu:
        if (key == KEY_RIGHT) {
            menuChange(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            ret =BUTTON_PAUSE_MS;
        }
        break;
    case OME_FunctionCall:;
        const void *retval;
        if (entry->fnPtr && key == KEY_RIGHT) {
            retval = entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            if (retval == MENU_CHAIN_BACK) {
                menuBack(displayPort);
            }
            if ((entry->flags & OME_REBOOT_REQUIRED)) {
                setRebootRequired();
            }
            ret =BUTTON_PAUSE_MS;
        }
        break;
    case OME_OSD_Exit:
        if (entry->fnPtr && key == KEY_RIGHT) {
            entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            ret =BUTTON_PAUSE_MS;
        }
        break;
#if defined(USE_OSD) && false
    case OME_VISIBLE:
        if (entry->data) {
            const auto* val = reinterpret_cast<uint16_t*>(entry->data);
            const uint16_t previousValue = *val;
            if ((key == KEY_RIGHT) && (!_elementEditing)) {
                _elementEditing = true;
                _osdProfileCursor = 1;
            } else if (_elementEditing) {
#if defined(USE_OSD_PROFILES)
                if (key == KEY_RIGHT) {
                    if (_osdProfileCursor < OSD_PROFILE_COUNT) {
                        ++_osdProfileCursor;
                    }
                }
                if (key == KEY_LEFT) {
                    if (_osdProfileCursor > 1) {
                        --_osdProfileCursor;
                    }
                }
#endif
                if (key == KEY_UP) {
                    setFlag(val, OSD_PROFILE_FLAG(_osdProfileCursor));
                }
                if (key == KEY_DOWN) {
                    clearFlag(val, OSD_PROFILE_FLAG(_osdProfileCursor));
                }
            }
            setFlag(_runtimeEntryFlags[_currentCtx.cursorRow], OME_PRINT_VALUE);

            if ((entry->flags & OME_REBOOT_REQUIRED) && (*val != previousValue)) {
                setRebootRequired();
            }
        }
        break;
#endif
    case OME_Bool:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT8_t*>(entry->data);
            const uint16_t previousValue = *ptr->val;
            *ptr->val = (key == KEY_RIGHT) ? 1 : 0;
            setFlag(_runtimeEntryFlags[_currentCtx.cursorRow], OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_INT8:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_INT8_t*>(entry->data);
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
            setFlag(_runtimeEntryFlags[_currentCtx.cursorRow], OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_UINT8:
        [[fallthrough]];
    case OME_FLOAT:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT8_t*>(entry->data);
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
            setFlag(_runtimeEntryFlags[_currentCtx.cursorRow], OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_UINT16:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT16_t*>(entry->data);
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
            setFlag(_runtimeEntryFlags[_currentCtx.cursorRow], OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_INT16:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_INT16_t*>(entry->data);
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
            setFlag(_runtimeEntryFlags[_currentCtx.cursorRow], OME_PRINT_VALUE);

            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_UINT32:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT32_t*>(entry->data);
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
            setFlag(_runtimeEntryFlags[_currentCtx.cursorRow], OME_PRINT_VALUE);

            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_INT32:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_INT32_t*>(entry->data);
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
            setFlag(_runtimeEntryFlags[_currentCtx.cursorRow], OME_PRINT_VALUE);

            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_TABLE:
        if ((entry->flags & OME_MASK) == OME_TABLE) {
            const auto* ptr = reinterpret_cast<const OSD_TABLE_t*>(entry->data);
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
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            }
            setFlag(_runtimeEntryFlags[_currentCtx.cursorRow], OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
        }
        break;
    case OME_String:
        [[fallthrough]];
    case OME_END:
        break;
    case OME_MENU:
        // Shouldn't happen
        break;
    }
    return ret;
//NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
}

CMSX::menu_t* CMSX::getSaveExitMenu()
{
    return nullptr;
}

void CMSX::menuOpen(DisplayPortBase& displayPort)
{
    const CMSX::menu_t* startMenu = _currentCtx.menu;
    if (_inMenu) {
        // Switch display
        DisplayPortBase* nextDisplayPort = _cms.displayPortSelectNext();
        if (nextDisplayPort == &displayPort) {
            return;
        }
        // DisplayPort has been changed.
        _currentCtx.cursorRow = cursorAbsolute();
        displayPort.setBackgroundType(DisplayPortBase::BACKGROUND_TRANSPARENT); // reset previous displayPort to transparent
        displayPort.release();
        _cms.setDisplayPort(nextDisplayPort);
    } else {
        //_displayPort = cmsDisplayPortSelectCurrent();
        //if (!_displayPort) {
        //    return;
        //}
        startMenu = &CMSX::menuMain;
        setInMenu(true);
        _currentCtx = { nullptr, 0, 0 };
        _menuStackIndex = 0;
        _cms.setArmingDisabled();
        displayPort.layerSelect(DisplayPortBase::LAYER_FOREGROUND);
    }
    displayPort.grab();
    //!!TODO: this should probably not have a dependency on the OSD or OSD slave code
#ifdef USE_OSD
    resumeRefreshAt = 0;
#endif

    const uint8_t columnCount = displayPort.getColumnCount();
    if (columnCount < NORMAL_SCREEN_MIN_COLS) {
        _smallScreen       = true;
        _linesPerMenuItem  = 2;
        _leftMenuColumn    = 0;
        _rightMenuColumn   = displayPort.getColumnCount();;
        _maxMenuItems      = (displayPort.getRowCount()) / _linesPerMenuItem;
    } else {
        _smallScreen       = false;
        _linesPerMenuItem  = 1;
        if (columnCount <= NORMAL_SCREEN_MAX_COLS) {
            _leftMenuColumn    = 2;
#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
            _rightMenuColumn   = columnCount - 2;
#else
            _rightMenuColumn   = columnCount - MENU_DRAW_BUFFER_LEN;
#endif
        } else {
            _leftMenuColumn    = (columnCount / 2) - 13;
#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
            _rightMenuColumn   = (columnCount / 2) + 13;
#else
            _rightMenuColumn   = columnCount - MENU_DRAW_BUFFER_LEN;
#endif
        }
        _maxMenuItems = displayPort.getRowCount() - 2;
    }

    if (displayPort.getUseFullScreen()) {
        _leftMenuColumn = 0;
        _rightMenuColumn   = columnCount;
        _maxMenuItems      = displayPort.getRowCount();
    }
    CMSX::menuChange(*this, displayPort, startMenu);
}

const void* CMSX::menuBack(DisplayPortBase& displayPort)
{
    // Let onExit function decide whether to allow exit or not.
    if (_currentCtx.menu->onExit) {
        const void* result = _currentCtx.menu->onExit(*this, displayPort, _pageTop + _currentCtx.cursorRow); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        if (result == MENU_CHAIN_BACK) {
            return result;
        }
    }
    _saveMenuInhibited = false;
    if (!_menuStackIndex) {
        return nullptr;
    }
    _currentCtx = _menuStack[--_menuStackIndex]; // pop
    menuCountPage();
    pageSelect(displayPort, _currentCtx.page);

    return nullptr;
}

const void* CMSX::menuChange(CMSX& cmsx, DisplayPortBase& displayPort, const menu_t* menu)
{
    if (!menu) {
        return nullptr;
    }
    if (menu == cmsx._currentCtx.menu) {
    } else {
        if (cmsx._currentCtx.menu && menu != &menuMain) {
            // If we are opening the initial top-level menu, then _currentCtx.menu will be NULL and there is nothing to do.
            // Otherwise stack the current menu before moving to the selected menu.
            if (cmsx._menuStackIndex >= MAX_MENU_STACK_DEPTH - 1) {
                // menu stack limit reached - prevent array overflow
                return nullptr;
            }
            cmsx._menuStack[cmsx._menuStackIndex++] = cmsx._currentCtx; // push
        }
        cmsx._currentCtx.menu = menu;
        cmsx._currentCtx.cursorRow = 0;

        if (menu->onEnter) {
            const void* result = menu->onEnter(cmsx, displayPort);
            if (result == MENU_CHAIN_BACK) {
                return menuBack(cmsx, displayPort, menu);
            }
        }

    }
    return nullptr;
}

void CMSX::menuCountPage()
{
    const OSD_Entry *entry = _currentCtx.menu->entries; 
    while ((entry->flags & OME_MASK) != OME_END) {
        ++entry; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    _pageCount = static_cast<uint8_t>(((entry - _currentCtx.menu->entries - 1) / _maxMenuItems) + 1);
}

const void* CMSX::menuBack(CMSX& cmsx, DisplayPortBase& displayPort, const menu_t* menu)
{
    (void)menu;

    if (cmsx._currentCtx.menu->onExit) {
        const void* result = cmsx._currentCtx.menu->onExit(cmsx, displayPort, cmsx._pageTop + cmsx._currentCtx.cursorRow); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        if (result == MENU_CHAIN_BACK) {
            return result;
        }
    }
    cmsx._saveMenuInhibited = false;
    if (cmsx._menuStackIndex == 0) {
        return nullptr;
    }
    cmsx._currentCtx = cmsx._menuStack[--cmsx._menuStackIndex]; // pop
    cmsx.menuCountPage();
    cmsx.pageSelect(displayPort, cmsx._currentCtx.page);

    return nullptr;
}

void CMSX::updateMaxRow()
{
    _pageMaxRow = 0;
    for (const OSD_Entry *entry = _pageTop; (entry->flags & OME_MASK) != OME_END; ++entry) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        ++_pageMaxRow;
    }
    if (_pageMaxRow > _maxMenuItems) {
        _pageMaxRow = _maxMenuItems;
    }
    if (_pageMaxRow > MAX_ROWS) {
        _pageMaxRow = MAX_ROWS;
    }
    --_pageMaxRow;
}

void CMSX::pageSelect(DisplayPortBase& displayPort, uint8_t newpage)
{
    _currentCtx.page = static_cast<uint8_t>((newpage + _pageCount) % _pageCount);
    _pageTop = &_currentCtx.menu->entries[_currentCtx.page * _maxMenuItems]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,bugprone-implicit-widening-of-multiplication-result)
    updateMaxRow();

    const OSD_Entry* entry = _pageTop;
    for (uint8_t ii = 0; ii <= _pageMaxRow; ++ii) {
        _runtimeEntryFlags[ii] = entry->flags;
        ++entry; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
}

void CMSX::pageNext(DisplayPortBase& displayPort)
{
    pageSelect(displayPort, _currentCtx.page + 1);
}

void CMSX::pagePrevious(DisplayPortBase& displayPort)
{
    pageSelect(displayPort, _currentCtx.page - 1);
}


// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast,hicpp-signed-bitwise,misc-no-recursion)
void CMSX::traverseGlobalExit(const CMSX::menu_t* menu)
{
    for (const CMSX::OSD_Entry* entry = menu->entries; (entry->flags & OME_MASK) != OME_END ; ++entry) {
        if ((entry->flags & OME_MASK) == OME_Submenu) {
            traverseGlobalExit(reinterpret_cast<const CMSX::menu_t*>(entry->data));
        }
    }

}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast,hicpp-signed-bitwise,misc-no-recursion)

const void* CMSX::menuExit(CMSX& cmsx, DisplayPortBase& displayPort, const menu_t* menu)
{
    if (menu == MENU_EXIT_SAVE || menu == MENU_EXIT_SAVE_REBOOT || menu == MENU_POPUP_SAVE || menu == MENU_POPUP_SAVE_REBOOT) {
        cmsx.traverseGlobalExit(&CMSX::menuMain);
        if (cmsx._currentCtx.menu->onExit) {
            cmsx._currentCtx.menu->onExit(cmsx, displayPort, nullptr); // Forced exit
        }
        if ((menu == MENU_POPUP_SAVE) || (menu == MENU_POPUP_SAVE_REBOOT)) {
            // traverse through the menu stack and call all their onExit functions
            for (int ii = cmsx._menuStackIndex - 1; ii >= 0; --ii) {
                if (cmsx._menuStack[static_cast<size_t>(ii)].menu->onExit) {
                    cmsx._menuStack[static_cast<size_t>(ii)].menu->onExit(cmsx, displayPort, nullptr);
                }
            }
        }
        //saveConfigAndNotify();
    }

    cmsx.setInMenu(false);
    displayPort.setBackgroundType(DisplayPortBase::BACKGROUND_TRANSPARENT);
    displayPort.release();
    cmsx._currentCtx.menu = nullptr;

    if ((menu == MENU_EXIT_SAVE_REBOOT) || (menu == MENU_POPUP_SAVE_REBOOT) || (menu == MENU_POPUP_EXIT_REBOOT)) {
        displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
        displayPort.writeString(5, 3, DisplayPortBase::SEVERITY_NORMAL, "REBOOTING...");
        displayPort.redraw();
#if false
        stopMotors();
        motorShutdown();
        delay(200);

        systemReset();
#endif
    }

    cmsx._cms.clearArmingDisabled();

    return nullptr;
}
