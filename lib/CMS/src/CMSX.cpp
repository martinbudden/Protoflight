#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "DisplayPortBase.h"
#include "OSD_Symbols.h"


//NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
const CMSX::menu_t* CMSX::MENU_NULL_PTR         = nullptr;
const CMSX::menu_t* CMSX::EXIT_PTR              = CMSX::MENU_NULL_PTR + 1;
const CMSX::menu_t* CMSX::EXIT_SAVE_PTR         = CMSX::MENU_NULL_PTR + 2;
const CMSX::menu_t* CMSX::EXIT_SAVE_REBOOT_PTR  = CMSX::MENU_NULL_PTR + 3;
const CMSX::menu_t* CMSX::POPUP_SAVE_PTR        = CMSX::MENU_NULL_PTR + 4;
const CMSX::menu_t* CMSX::POPUP_SAVE_REBOOT_PTR = CMSX::MENU_NULL_PTR + 5;
const CMSX::menu_t* CMSX::POPUP_EXIT_REBOOT_PTR = CMSX::MENU_NULL_PTR + 6;
//NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)


CMSX::CMSX(CMS& cms) :
    _cms(cms)
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
    return _currentCtx.cursorRow + _currentCtx.page * _maxMenuItems;
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
    if ((row->flags == DYNAMIC) || rowSliderOverride(row->flags)) {
        if (type == OME_UINT8  || type == OME_INT8 || type == OME_UINT16  || type == OME_INT16) {
            return true;
        }
    }
    return false;
}

void CMSX::drawMenu(DisplayPortBase& displayPort, uint32_t currentTimeUs)
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

    uint8_t i = 0;
    if (displayWasCleared) {
        for (const OSD_Entry* p = _pageTop; (p <= _pageTop + _pageMaxRow); p++, i++) {
            _runtimeEntryFlags[i] |= PRINT_LABEL;
            _runtimeEntryFlags[i] |= PRINT_VALUE;
        }
    } else if (drawPolled) {
        for (const OSD_Entry* p = _pageTop; (p <= _pageTop + _pageMaxRow); p++, i++) {
            if (p->flags & DYNAMIC) {
                _runtimeEntryFlags[i] |= PRINT_VALUE;
            }
        }
    }
    while (rowIsSkippable(_pageTop + _currentCtx.cursorRow)) { // skip labels, strings and dynamic read-only entries
        ++_currentCtx.cursorRow;
    }

    //const uint8_t top = smallScreen ? 1 : (pDisplay->rows - pageMaxRow)/2;
    const uint8_t top = 1;

    if (_currentCtx.cursorRow != displayPort.getCursorRow()) {
        room -= displayPort.writeString(_leftMenuColumn, top + displayPort.getCursorRow() * _linesPerMenuItem, DisplayPortBase::SEVERITY_NORMAL, " ");
    }

    if (room < 30) {
        return;
    }

    if (displayPort.getCursorRow() != _currentCtx.cursorRow) {
        room -= displayPort.writeString(_leftMenuColumn, top + _currentCtx.cursorRow * _linesPerMenuItem, DisplayPortBase::SEVERITY_NORMAL, ">");
        displayPort.setCursorRow(_currentCtx.cursorRow);
    }

    if (room < 30) {
        return;
    }

    if (_currentCtx.menu->onDisplayUpdate) {
        const void* result = _currentCtx.menu->onDisplayUpdate(*this, displayPort, _pageTop + _currentCtx.cursorRow);
        if (result == &_menuChainBack) {
            menuBack(*this, displayPort, nullptr);
            return;
        }
    }
    // Print text labels
    i = 0;
    for (const OSD_Entry* p = _pageTop; (p <= _pageTop + _pageMaxRow); i++, p++) {
        if (_runtimeEntryFlags[i] & PRINT_LABEL) {
            uint8_t coloff = _leftMenuColumn;
            coloff += ((p->flags & OME_MASK) == OME_Label) ? 0 : 1;
            room -= displayPort.writeString(coloff, top + i * _linesPerMenuItem, DisplayPortBase::SEVERITY_NORMAL, p->text);
            _runtimeEntryFlags[i] &= ~PRINT_LABEL;
            if (room < 30) {
                return;
            }
        }
        // Highlight values overridden by sliders
        if (rowSliderOverride(p->flags)) {
            displayPort.writeChar(_leftMenuColumn - 1, top + i * _linesPerMenuItem, DisplayPortBase::SEVERITY_NORMAL, 'S');
        }

    // Print values

    // XXX Polled values at latter positions in the list may not be
    // XXX printed if not enough room in the middle of the list.

        if ((_runtimeEntryFlags[i] & PRINT_VALUE) || (_runtimeEntryFlags[i] & SCROLLING_TICKER)) {
            bool selectedRow = (i == _currentCtx.cursorRow);
            room -= drawMenuEntry(displayPort, p, top + i * _linesPerMenuItem, selectedRow, &_runtimeEntryFlags[i], &_runtimeTableTicker[i]);
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
            const uint8_t symbol = displayPort.supportsOsdSymbols() ? SYM_ARROW_SMALL_DOWN : 'v';
            displayPort.writeChar(_leftMenuColumn - 1, top + _pageMaxRow, DisplayPortBase::SEVERITY_NORMAL, symbol);
        }
    }

}

int CMSX::drawMenuEntry(DisplayPortBase& displayPort, const OSD_Entry *p, uint8_t row, bool selectedRow, uint8_t *flags, table_ticker_t* ticker)
{
    (void)displayPort;
    (void)p;
    (void)row;
    (void)selectedRow;
    (void)flags;
    (void)ticker;

    return 0;
}


void CMSX::addMenuEntry(CMSX::OSD_Entry& menuEntry, const char* text, uint32_t flags, CMSX::entryFnPtr fnPtr, void* data)
{
    menuEntry.text = text;
    menuEntry.flags = flags;
    menuEntry.fnPtr = fnPtr;
    menuEntry.data = data;
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
            const void* result = menu->onEnter(cmsx, displayPort, nullptr);
            if (result == &cmsx._menuChainBack) {
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
    _pageCount = static_cast<int8_t>(((entry - _currentCtx.menu->entries - 1) / _maxMenuItems) + 1);
}

const void* CMSX::menuBack(CMSX& cmsx, DisplayPortBase& displayPort, const menu_t* menu)
{
    (void)menu;

    if (cmsx._currentCtx.menu->onExit) {
        const void* result = cmsx._currentCtx.menu->onExit(cmsx, displayPort, cmsx._pageTop + cmsx._currentCtx.cursorRow); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        if (result == &cmsx._menuChainBack) {
            return result;
        }
    }
    cmsx._saveMenuInhibited = false;
    if (!cmsx._menuStackIndex) {
        return nullptr;
    }
    cmsx._currentCtx = cmsx._menuStack[--cmsx._menuStackIndex]; // pop
    cmsx.menuCountPage();
    cmsx.pageSelect(displayPort, static_cast<int8_t>(cmsx._currentCtx.page));

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

void CMSX::pageSelect(DisplayPortBase& displayPort, int8_t newpage)
{
    _currentCtx.page = (newpage + _pageCount) % _pageCount;
    _pageTop = &_currentCtx.menu->entries[_currentCtx.page * _maxMenuItems]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,bugprone-implicit-widening-of-multiplication-result)
    updateMaxRow();

    const OSD_Entry* entry = _pageTop;
    for (uint8_t ii = 0; ii <= _pageMaxRow; ++ii) {
        _runtimeEntryFlags[ii] = entry->flags;
        ++entry; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
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
    if (menu == EXIT_SAVE_PTR || menu == EXIT_SAVE_REBOOT_PTR || menu == POPUP_SAVE_PTR || menu == POPUP_SAVE_REBOOT_PTR) {
        cmsx.traverseGlobalExit(&CMSX::menuMain);
        if (cmsx._currentCtx.menu->onExit) {
            cmsx._currentCtx.menu->onExit(cmsx, displayPort, nullptr); // Forced exit
        }
        if ((menu == POPUP_SAVE_PTR) || (menu == POPUP_SAVE_REBOOT_PTR)) {
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

    if ((menu == EXIT_SAVE_REBOOT_PTR) || (menu == POPUP_SAVE_REBOOT_PTR) || (menu == POPUP_EXIT_REBOOT_PTR)) {
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

    //unsetArmingDisabled(ARMING_DISABLED_CMS_MENU);

    return nullptr;
}
