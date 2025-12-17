#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "DisplayPortBase.h"
#include "FormatInteger.h"
#include "OSD_Elements.h"
#include <algorithm>
#include <cstring>


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


CMSX::CMSX(CMS& cms, IMU_Filters& imuFilters, IMU_Base& imu, VTX* vtx) :
    _cms(cms),
    _imuFilters(imuFilters),
    _imu(imu),
    _vtx(vtx),
    _menuMain(menuMain)
    //_menuMain(menuFilters)
    //_menuMain(menuRates)
    //_menuMain(menuRcPreview)
{
}

Cockpit& CMSX::getCockpit()
{
    return _cms.getCockpit();
}

const Cockpit& CMSX::getCockpit() const
{
    return _cms.getCockpit();
}

void CMSX::setRebootRequired()
{
    getCockpit().setRebootRequired();
}

bool CMSX::getRebootRequired() const
{
    return getCockpit().getRebootRequired();
}

CMSX::menu_t* CMSX::getSaveExitMenu()
{
    if (getRebootRequired()) {
        return &CMSX::menuSaveExitReboot;
    }
    return &CMSX::menuSaveExit;
}

bool CMSX::setupPopupMenuBuild()
{
    uint8_t menuIndex = 0;

    menuSetupPopupEntries[menuIndex] = { "-- SETUP MENU --", OME_LABEL, nullptr, nullptr };
    // Add menu entries for uncompleted setup tasks
    if (getCockpit().getArmingDisableFlags() & ARMING_DISABLED_ACC_CALIBRATION) { // NOLINT(hicpp-signed-bitwise)
        menuSetupPopupEntries[++menuIndex] = { "CALIBRATE ACC", OME_FUNCTION_CALL | OME_DYNAMIC, menuCalibrateAcc, &AccCalibrationStatus[0] };
    }
#if defined(USE_BATTERY_CONTINUE)
    if (hasUsedMAh()) {
        cmsAddMenuEntry(&setupPopupMenuEntries[++menuIndex], batteryContinueAmount, OME_Funcall | DYNAMIC, cmsRestoreMah, nullptr);
    }
#endif
    menuSetupPopupEntries[++menuIndex] = { "EXIT", OME_BACK | OME_DYNAMIC, nullptr, nullptr };
    menuSetupPopupEntries[++menuIndex] = { "NULL", OME_END, nullptr, nullptr };

    return (menuIndex > 2);  // return true if any setup items were added
}

// Check if overridden by slider, used by simplified PID tuning
bool CMSX::rowSliderOverride(const uint16_t flags)
{
    (void)flags;
    return false;
}

bool CMSX::rowIsSkippable(const OSD_Entry* row)
{
    const uint16_t entryType = row->flags & OME_TYPE_MASK;

    if (entryType == OME_LABEL) {
        return true;
    }
    if (entryType == OME_STRING) {
        return true;
    }
    if ((row->flags == OME_DYNAMIC) || rowSliderOverride(row->flags)) { // cppcheck-suppress knownConditionTrueFalse
        if (entryType == OME_UINT8  || entryType == OME_INT8 || entryType == OME_UINT16  || entryType == OME_INT16) {
            return true;
        }
    }
    return false;
}

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
    auto len = static_cast<uint8_t>(strnlen(buf, size));

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
    if (_rightAligned) {
        padLeft(buf, size);
    } else {
        padRight(buf, size);
    }
}

uint32_t CMSX::drawMenuItemValue(DisplayPortBase& displayPort, uint8_t row, uint8_t maxSize) // NOLINT(readability-make-member-function-const)
{
    padToSize(&_menuDrawBuf[0], maxSize < MENU_DRAW_BUFFER_LEN ? maxSize : MENU_DRAW_BUFFER_LEN);
    const uint8_t column = _rightAligned ? _rightMenuColumn - maxSize : _rightMenuColumn;
    return displayPort.writeString(column, row, &_menuDrawBuf[0]);
}

uint32_t CMSX::drawMenuTableItemValue(DisplayPortBase& displayPort, uint8_t row, uint8_t maxSize) // NOLINT(readability-make-member-function-const)
{
    padToSize(&_menuTableBuf[0], maxSize < MENU_TABLE_BUFFER_LEN ? maxSize : MENU_TABLE_BUFFER_LEN);
    const uint8_t column = _rightAligned ? _rightMenuColumn - maxSize : _rightMenuColumn;
    return displayPort.writeString(column, row, &_menuTableBuf[0]);
}

uint32_t CMSX::drawMenuTableEntry(DisplayPortBase& displayPort, const OSD_Entry* entry, uint8_t row, uint16_t& entryFlags, table_ticker_t& ticker)
{
    uint32_t count = 0;

    if (((entryFlags & OME_PRINT_VALUE) || (entryFlags & OME_SCROLLING_TICKER)) && entry->data) {
        const auto* ptr = reinterpret_cast<const OSD_TABLE_t*>(entry->data); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        //const size_t labelLength = strnlen(entry->text , MENU_DRAW_BUFFER_LEN) + 1; // account for the space between label and display data
        const uint8_t index = std::clamp(*ptr->val, static_cast<uint8_t>(0), ptr->max);
        const char* str = static_cast<const char *>(ptr->names[index]);   // lookup table display text NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        const uint8_t displayLength = static_cast<uint8_t>(std::strlen(str));
        const uint8_t availableSpace = displayPort.getColumnCount() - _rightMenuColumn;

        bool drawText = false;
        if (entryFlags & OME_PRINT_VALUE) {
            drawText = true;
            ticker.state = 0;
            ticker.loopCounter = 0;
            if (displayLength > availableSpace) { // table entry text is longer than the available space so start the ticker
                setFlag(entryFlags, OME_SCROLLING_TICKER);
            } else {
                clearFlag(entryFlags, OME_SCROLLING_TICKER);
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
            count += drawMenuTableItemValue(displayPort, row, availableSpace);
        }
        clearFlag(entryFlags, OME_PRINT_VALUE);
    }

    return count;
}

uint32_t CMSX::drawMenuEntry(DisplayPortBase& displayPort, const OSD_Entry* entry, uint8_t row, uint16_t& entryFlags, table_ticker_t& ticker) // NOLINT(readability-function-cognitive-complexity)
{
    const uint16_t entryType = entry->flags & OME_TYPE_MASK;
    uint32_t count = 0;
    if (entryFlags & OME_PRINT_LABEL) {
        const uint8_t column = _leftMenuColumn + ((entryType == OME_LABEL) ? 0 : 1);
        count += displayPort.writeString(column, row, entry->text);
        clearFlag(entryFlags, OME_PRINT_LABEL);
    }

    if (entryType == OME_TABLE) {
        count += drawMenuTableEntry(displayPort, entry, row, entryFlags, ticker);
        return count;
    };

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-pointer-arithmetic)
    switch (entryType) {
    case OME_LABEL:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            // A label with optional string, immediately following text
            strncpy(&_menuDrawBuf[0], reinterpret_cast<const char*>(entry->data), MENU_DRAW_BUFFER_LEN);
            count += drawMenuItemValue(displayPort, row, static_cast<uint8_t>(strnlen(&_menuDrawBuf[0], MENU_DRAW_BUFFER_LEN)));
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_SUBMENU:
        [[fallthrough]];
    case OME_FUNCTION_CALL:
        if ((entryFlags & OME_PRINT_VALUE)) {
            _menuDrawBuf[0] = 0;
            if (entryType == OME_SUBMENU && entry->fnPtr && entryFlags & OME_OPTION_STRING) {
                // Special case of sub menu entry with optional value display.
                const char *str = reinterpret_cast<const char*>(entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data)));
                strncpy(&_menuDrawBuf[0], str, MENU_DRAW_BUFFER_LEN);
            } else if (entryType == OME_FUNCTION_CALL && entry->data) {
                strncpy(&_menuDrawBuf[0], reinterpret_cast<const char*>(entry->data), MENU_DRAW_BUFFER_LEN);
            }
            strncat(&_menuDrawBuf[0], ">", MENU_DRAW_BUFFER_LEN);
            row = _smallScreen ? row - 1 : row;
            count += drawMenuItemValue(displayPort, row, static_cast<uint8_t>(strnlen(&_menuDrawBuf[0], MENU_DRAW_BUFFER_LEN)));
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_STRING:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            strncpy(reinterpret_cast<char*>(&_menuDrawBuf[0]), static_cast<const char*>(entry->data), MENU_DRAW_BUFFER_LEN);
            count += drawMenuItemValue(displayPort, row, MENU_DRAW_BUFFER_LEN);
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_BOOL:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_BOOL_t*>(entry->data);
            _menuDrawBuf[0] = *ptr->val ? '1' : '0';
            _menuDrawBuf[1] = 0;
            count += drawMenuItemValue(displayPort, row, NUMBER_FIELD_LEN);
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_UINT8:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT8_t*>(entry->data);
            ui2a(*ptr->val, &_menuDrawBuf[0]);
            count += drawMenuItemValue(displayPort, row, NUMBER_FIELD_LEN);
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_UINT8_FIXED:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT8_FIXED_t*>(entry->data);
            formatFixed6point3(*ptr->val*ptr->multiplier, &_menuDrawBuf[0]);
            ui2a(*ptr->val, &_menuDrawBuf[0]);
            count += drawMenuItemValue(displayPort, row, NUMBER_FIELD_LEN);
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_INT8:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_INT8_t*>(entry->data);
            i2a(*ptr->val, &_menuDrawBuf[0]);
            count += drawMenuItemValue(displayPort, row, NUMBER_FIELD_LEN);
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_UINT16:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT16_t*>(entry->data);
            ui2a(*ptr->val, &_menuDrawBuf[0]);
            count += drawMenuItemValue(displayPort, row, NUMBER_FIELD_LEN);
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_UINT16_FIXED:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT16_FIXED_t*>(entry->data);
            formatFixed6point3(*ptr->val*ptr->multiplier, &_menuDrawBuf[0]);
            ui2a(*ptr->val, &_menuDrawBuf[0]);
            count += drawMenuItemValue(displayPort, row, NUMBER_FIELD_LEN);
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_INT16:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_INT16_t*>(entry->data);
            i2a(*ptr->val, &_menuDrawBuf[0]);
            count += drawMenuItemValue(displayPort, row, NUMBER_FIELD_LEN);
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_UINT32:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT32_t*>(entry->data);
            ui2a(*ptr->val, &_menuDrawBuf[0]);
            count += drawMenuItemValue(displayPort, row, NUMBER_FIELD_LEN);
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    case OME_INT32:
        if ((entryFlags & OME_PRINT_VALUE) && entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_INT32_t*>(entry->data);
            i2a(*ptr->val, &_menuDrawBuf[0]);
            count += drawMenuItemValue(displayPort, row, NUMBER_FIELD_LEN);
            clearFlag(entryFlags, OME_PRINT_VALUE);
        }
        break;
    default:
        break;
    }
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return count;
}

void CMSX::drawMenu(DisplayPortBase& displayPort, uint32_t currentTimeUs) // NOLINT(readability-function-cognitive-complexity)
{
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    if (!_pageTop || !_inMenu) {
        return;
    }
    const bool displayWasCleared = displayPort.isCleared();
    displayPort.setCleared(false);

    bool drawDynamicValues = false;
    if (currentTimeUs > _lastPolledUs + DYNAMIC_VALUES_POLLING_INTERVAL_US) {
        drawDynamicValues = true;
        _lastPolledUs = currentTimeUs;
    }

    if (displayWasCleared) {
        //Serial.printf("drawMenu setting entryFlags\r\n"); // called
        uint8_t ii = 0;
        for (const OSD_Entry* entry = _pageTop; entry <= _pageTop + _pageMaxRow; ++entry, ++ii) {
            _entryFlags[ii] |= OME_PRINT_LABEL | OME_PRINT_VALUE;
        }
    } else if (drawDynamicValues) {
        //Serial.printf("drawMenu drawDynamicValues\r\n");
        uint8_t ii = 0;
        for (const OSD_Entry* entry = _pageTop; entry <= _pageTop + _pageMaxRow; ++entry, ++ii) {
            if (entry->flags & OME_DYNAMIC) {
                _entryFlags[ii] |= OME_PRINT_VALUE;
            }
        }
    }

    // skip labels, strings, and dynamic read-only entries
    while (rowIsSkippable(_pageTop + _currentMenuContext.cursorRow)) {
        ++_currentMenuContext.cursorRow;
    }

    // position the menu in the bottom half of the screen
    const uint8_t topRow = _smallScreen ? 1 : static_cast<uint8_t>((displayPort.getRowCount() - _pageMaxRow)/2);
    auto spaceLeft = static_cast<int32_t>(displayPort.txBytesFree());

    if (_cursorRow != _currentMenuContext.cursorRow) {
        // cursor position has changed, so:
        // clear the old cursor
        if (_cursorRow != CURSOR_ROW_NOT_SET) {
            const uint8_t row = topRow + static_cast<uint8_t>(_cursorRow * _linesPerMenuItem);
            spaceLeft -= static_cast<int32_t>(displayPort.writeString(_leftMenuColumn, row, " "));
        }
        // and draw the new one
        _cursorRow = _currentMenuContext.cursorRow;
    }
    const uint8_t row = topRow + static_cast<uint8_t>(_cursorRow * _linesPerMenuItem);
    spaceLeft -= static_cast<int32_t>(displayPort.writeString(_leftMenuColumn, row, ">"));
    if (_currentMenuContext.menu->onDisplayUpdate) {
        if (_currentMenuContext.menu->onDisplayUpdate(*this, displayPort, _pageTop + _currentMenuContext.cursorRow) == MENU_BACK) {
            menuBack(displayPort, nullptr);
            return;
        }
    }

    // Display the menu entries
    uint8_t ii = 0;
    for (const OSD_Entry* entry = _pageTop; entry <= _pageTop + _pageMaxRow; ++ii, ++entry) {
        const uint8_t entryRow = topRow + static_cast<uint8_t>(ii * _linesPerMenuItem);
        // display the current item indicator
        //uint16_t& entryFlags = _entryFlags[ii];
        // Highlight values overridden by sliders
        if (rowSliderOverride(entry->flags)) { // cppcheck-suppress knownConditionTrueFalse
            displayPort.writeChar(_leftMenuColumn - 1, entryRow, 'S');
        }
        // Print values
        // XXX Polled values at latter positions in the list may not be
        // XXX printed if not enough room in the middle of the list.
        if ((_entryFlags[ii] & OME_PRINT_VALUE) || (_entryFlags[ii] & OME_SCROLLING_TICKER)) {
            //const bool selectedRow = (ii == _currentMenuContext.cursorRow);
            spaceLeft -= static_cast<int32_t>(drawMenuEntry(displayPort, entry, entryRow, _entryFlags[ii], _runtimeTableTicker[ii]));
            enum { CHARACTERS_PER_LINE };
            if (spaceLeft < CHARACTERS_PER_LINE) {
                return;
            }
        }
    }

    // Draw the up/down page indicators if the display has space.
    // Only draw the symbols when necessary after the screen has been cleared. Otherwise they're static.
    if (displayWasCleared && _leftMenuColumn > 0) { // make sure there's room to draw the symbol
        if (_currentMenuContext.page > 0) {
            displayPort.writeChar(_leftMenuColumn - 1, topRow, displayPort.getSmallArrowUp());
        }
        if (_currentMenuContext.page < _pageCount - 1) {
            displayPort.writeChar(_leftMenuColumn - 1, topRow + _pageMaxRow, displayPort.getSmallArrowDown());
        }
    }
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

uint16_t CMSX::handleKey(DisplayPortBase& displayPort, key_e key) // NOLINT(readability-function-cognitive-complexity)
{
    if (!_currentMenuContext.menu) {
        return BUTTON_TIME_MS;
    }
    if (key == KEY_MENU) {
        menuOpen(displayPort);
        return BUTTON_PAUSE_MS;
    }
    if (key == KEY_ESC) {
        if (_elementEditing) {
            _elementEditing = false;
        } else {
            menuBack(displayPort, nullptr);
        }
        return BUTTON_PAUSE_MS;
    }
    if (key == KEY_SAVE_MENU && !_saveMenuInhibited) {
        _elementEditing = false;
        menuChange(*this, displayPort, getSaveExitMenu());
        return BUTTON_PAUSE_MS;
    }
    if (!_elementEditing) {
        if (key == KEY_DOWN) {
            if (_currentMenuContext.cursorRow < _pageMaxRow) {
                ++_currentMenuContext.cursorRow;
            } else {
                pageNext();
                displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
                _currentMenuContext.cursorRow = 0;    // Goto top in any case
            }
            return BUTTON_TIME_MS;
        }
        if (key == KEY_UP) {
            auto cursorRow = static_cast<int8_t>(_currentMenuContext.cursorRow);
            --cursorRow;
            // Skip non-title labels, strings and dynamic read-only entries
            while ((rowIsSkippable(_pageTop + _currentMenuContext.cursorRow)) && cursorRow > 0) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
                --cursorRow;
            }
            if (cursorRow == -1 || ((_pageTop + cursorRow)->flags & OME_TYPE_MASK) == OME_LABEL) { //NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
                // Goto previous page
                pagePrevious();
                displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
                _currentMenuContext.cursorRow = _pageMaxRow;
            } else {
                _currentMenuContext.cursorRow = static_cast<uint8_t>(cursorRow);
            }
            return BUTTON_TIME_MS;
        }
    }
    const OSD_Entry* entry = _pageTop + _currentMenuContext.cursorRow; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return handleKey(displayPort, key, entry, _entryFlags[_currentMenuContext.cursorRow]);
}

uint16_t CMSX::handleKey(DisplayPortBase& displayPort, key_e key, const OSD_Entry* entry, uint16_t& entryFlags) // NOLINT(readability-function-cognitive-complexity)
{
//NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
    uint16_t ret = BUTTON_TIME_MS;

    switch (entry->flags & OME_TYPE_MASK) {
    case OME_LABEL:
        break;
    case OME_SUBMENU:
        if (key == KEY_RIGHT) {
            menuChange(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            ret = BUTTON_PAUSE_MS;
        }
        break;
    case OME_FUNCTION_CALL:
        if (entry->fnPtr && key == KEY_RIGHT) {
            if (entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data)) == MENU_BACK) {
                menuBack(displayPort, nullptr);
            }
            if ((entry->flags & OME_REBOOT_REQUIRED)) {
                setRebootRequired();
            }
            ret = BUTTON_PAUSE_MS;
        }
        break;
    case OME_EXIT:
        if (entry->fnPtr && key == KEY_RIGHT) {
            entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            ret = BUTTON_PAUSE_MS;
        }
        break;
    case OME_BACK:
        menuBack(displayPort, nullptr);
        ret = BUTTON_PAUSE_MS;
        _elementEditing = false;
        break;
    case OME_END:
        break;
#if defined(USE_OSD)
    case OME_VISIBLE:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_UINT16_t*>(entry->data);
            const uint16_t previousValue = *ptr->val;
            if ((key == KEY_RIGHT) && (!_elementEditing)) {
                _elementEditing = true;
                _osdProfileCursor = 0;
            } else if (_elementEditing) {
                if (key == KEY_RIGHT) {
                    if (_osdProfileCursor < OSD_Elements::PROFILE_COUNT) {
                        ++_osdProfileCursor;
                    }
                }
                if (key == KEY_LEFT) {
                    if (_osdProfileCursor > 0) {
                        --_osdProfileCursor;
                    }
                }
                if (key == KEY_UP) {
                    setFlag(*ptr->val, OSD_Elements::profileFlag(_osdProfileCursor));
                }
                if (key == KEY_DOWN) {
                    clearFlag(*ptr->val, OSD_Elements::profileFlag(_osdProfileCursor));
                }
            }
            setFlag(entryFlags, OME_PRINT_VALUE);

            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
        }
        break;
#endif
    case OME_STRING:
        break;
    case OME_TABLE:
        if (entry->data) {
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
            setFlag(entryFlags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
        }
        break;
    case OME_BOOL:
        if (entry->data) {
            const auto* ptr = reinterpret_cast<const OSD_BOOL_t*>(entry->data);
            const bool previousValue = *ptr->val;
            *ptr->val = (key == KEY_RIGHT) ? true : false;
            setFlag(entryFlags, OME_PRINT_VALUE);
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
            setFlag(entryFlags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_UINT8_FIXED:
        [[fallthrough]];
    case OME_UINT8:
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
            setFlag(entryFlags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
            }
        }
        break;
    case OME_UINT16_FIXED:
        [[fallthrough]];
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
            setFlag(entryFlags, OME_PRINT_VALUE);
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
            setFlag(entryFlags, OME_PRINT_VALUE);

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
            setFlag(entryFlags, OME_PRINT_VALUE);
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
            setFlag(entryFlags, OME_PRINT_VALUE);
            if ((entry->flags & OME_REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                setRebootRequired();
            }
            if (entry->fnPtr) {
                entry->fnPtr(*this, displayPort, reinterpret_cast<const menu_t*>(entry->data));
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

void CMSX::menuOpen(DisplayPortBase& displayPort)
{
    const CMSX::menu_t* startMenu = _currentMenuContext.menu;
    if (_inMenu) {
        // Switch display
        //DisplayPortBase* nextDisplayPort = _cms.displayPortSelectNext();
        //if (nextDisplayPort == &displayPort) {
        //    return;
        //}
        // DisplayPort has been changed.
        //_currentMenuContext.cursorRow += static_cast<uint8_t>(_currentMenuContext.page * _maxMenuItems);
        displayPort.setBackgroundType(DisplayPortBase::BACKGROUND_TRANSPARENT); // reset previous displayPort to transparent
        //displayPort.release();
        //_cms.setDisplayPort(nextDisplayPort);
    } else {
        //_displayPort = cmsDisplayPortSelectCurrent();
        //if (!_displayPort) {
        //    return;
        //}
        _inMenu = true;
        startMenu = &_menuMain;
        _currentMenuContext = { nullptr, 0, 0 };
        menuStackReset();
        _cms.setArmingDisabled();
        _cursorRow = CURSOR_ROW_NOT_SET;
        displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
        displayPort.grab();
        displayPort.layerSelect(DisplayPortBase::LAYER_FOREGROUND);
    }
    //!!TODO: this should not have a dependency on the OSD
#if defined(USE_OSD)
    //!!_resumeRefreshAtUs = 0;
#endif

    const uint8_t columnCount = displayPort.getColumnCount();
    if (columnCount < NORMAL_SCREEN_MIN_COLS) {
        _smallScreen       = true;
        _rightAligned      = true;
        _linesPerMenuItem  = 2;
        _leftMenuColumn    = 0;
        _rightMenuColumn   = columnCount;
        _maxMenuItems      = displayPort.getRowCount() / _linesPerMenuItem;
    } else {
        _smallScreen       = false;
        _rightAligned      = true;
        _linesPerMenuItem  = 1;
        if (columnCount <= NORMAL_SCREEN_MAX_COLS) {
            _leftMenuColumn    = 1;
            _rightMenuColumn   = _rightAligned ? columnCount - 2 : columnCount - MENU_DRAW_BUFFER_LEN; // cppcheck-suppress knownConditionTrueFalse
        } else {
            _leftMenuColumn    = (columnCount / 2) - 13;
            _rightMenuColumn   = _rightAligned ? (columnCount / 2) + 13 : columnCount - MENU_DRAW_BUFFER_LEN;
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

const void* CMSX::menuChange(DisplayPortBase& displayPort, const menu_t* menu)
{
    if (!menu) {
        return nullptr;
    }
    if (menu == _currentMenuContext.menu) {
        const uint8_t cursorAbs = _currentMenuContext.cursorRow;
       _currentMenuContext.cursorRow = cursorAbs %_maxMenuItems;
       pageSelect(cursorAbs /_maxMenuItems);
        displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
    } else {
        if (_currentMenuContext.menu && menu != &_menuMain) {
            // If we are opening the initial top-level menu, then _currentMenuContext.menu will be nullptr and there is nothing to do.
            // Otherwise stack the current menu before moving to the selected menu.
            if (menuStackPush() == MENU_STACK_NO_ROOM_TO_PUSH) {
                return nullptr;
            }
        }
       _currentMenuContext.menu = menu;
       _currentMenuContext.cursorRow = 0;
        if (menu->onEnter) {
            if (menu->onEnter(*this, displayPort) == MENU_BACK) {
                return menuBack(displayPort, menu);
            }
        }
        pageSelect(0);
        displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
    }
    return nullptr;
}

const void* CMSX::menuBack(DisplayPortBase& displayPort, const menu_t* menu)
{
    (void)menu;

    if (_currentMenuContext.menu->onExit) {
        if (_currentMenuContext.menu->onExit(*this, displayPort,_pageTop +_currentMenuContext.cursorRow) == MENU_BACK) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            return MENU_BACK;
        }
    }
   _saveMenuInhibited = false;
    if (menuStackPop() == MENU_STACK_NOTHING_TO_POP) {
        return nullptr;
    }
    pageSelect(_currentMenuContext.page);
    displayPort.clearScreen(DISPLAY_CLEAR_WAIT);

    return nullptr;
}

const void* CMSX::menuExit(DisplayPortBase& displayPort, const menu_t* menu)
{
    if (menu == MENU_EXIT_SAVE || menu == MENU_EXIT_SAVE_REBOOT || menu == MENU_POPUP_SAVE || menu == MENU_POPUP_SAVE_REBOOT) {
        traverseGlobalExit(&_menuMain);
        if (_currentMenuContext.menu->onExit) {
            _currentMenuContext.menu->onExit(*this, displayPort, nullptr); // Forced exit
        }
        if ((menu == MENU_POPUP_SAVE) || (menu == MENU_POPUP_SAVE_REBOOT)) {
            // traverse through the menu stack and call all their onExit functions
            for (int ii = _menuStackIndex - 1; ii >= 0; --ii) {
                if (_menuStack[static_cast<size_t>(ii)].menu->onExit) {
                   _menuStack[static_cast<size_t>(ii)].menu->onExit(*this, displayPort, nullptr);
                }
            }
        }
        //saveConfigAndNotify();
    }

    displayPort.setBackgroundType(DisplayPortBase::BACKGROUND_TRANSPARENT);
   _inMenu = false;
    displayPort.release();
   _currentMenuContext.menu = nullptr;

    if ((menu == MENU_EXIT_SAVE_REBOOT) || (menu == MENU_POPUP_SAVE_REBOOT) || (menu == MENU_POPUP_EXIT_REBOOT)) {
        _cursorRow = CURSOR_ROW_NOT_SET;
        displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
        displayPort.writeString(5, 3, "REBOOTING...");
        displayPort.redraw();
#if false
        stopMotors();
        motorShutdown();
        delay(200);

        systemReset();
#endif
    }

   _cms.clearArmingDisabled();

    return nullptr;
}

void CMSX::pageSelect(uint8_t newpage)
{
    const OSD_Entry* menuEntry = _currentMenuContext.menu->entries;
    while ((menuEntry->flags & OME_TYPE_MASK) != OME_END) {
        ++menuEntry; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    _pageCount = static_cast<uint8_t>(((menuEntry - _currentMenuContext.menu->entries - 1) / _maxMenuItems) + 1);

    _currentMenuContext.page = static_cast<uint8_t>((newpage + _pageCount) % _pageCount);
    _pageTop = &_currentMenuContext.menu->entries[_currentMenuContext.page * _maxMenuItems]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,bugprone-implicit-widening-of-multiplication-result)

    uint8_t ii = 0;
    for (const OSD_Entry* entry = _pageTop; (entry->flags & OME_TYPE_MASK) != OME_END; ++entry) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        _entryFlags[ii] = entry->flags;
        ++ii;
    }
    _pageMaxRow = ii;
    if (_pageMaxRow > _maxMenuItems) {
        _pageMaxRow = _maxMenuItems;
    }
    if (_pageMaxRow > MAX_ROWS) {
        _pageMaxRow = MAX_ROWS;
    }
    --_pageMaxRow;
    _cursorRow = CURSOR_ROW_NOT_SET;
}

void CMSX::pageNext()
{
    pageSelect(_currentMenuContext.page + 1);
}

void CMSX::pagePrevious()
{
    pageSelect(_currentMenuContext.page - 1);
}

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast,hicpp-signed-bitwise,misc-no-recursion)
void CMSX::traverseGlobalExit(const CMSX::menu_t* menu)
{
    for (const CMSX::OSD_Entry* entry = menu->entries; (entry->flags & OME_TYPE_MASK) != OME_END ; ++entry) {
        if ((entry->flags & OME_TYPE_MASK) == OME_SUBMENU) {
            traverseGlobalExit(reinterpret_cast<const CMSX::menu_t*>(entry->data));
        }
    }
}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast,hicpp-signed-bitwise,misc-no-recursion)

void CMSX::menuStackReset()
{
    _menuStackIndex = 0;
}

CMSX::menu_stack_e CMSX::menuStackPush() // NOLINT(readability-make-member-function-const)
{
    if (_menuStackIndex >= MAX_MENU_STACK_DEPTH - 1) {
         return MENU_STACK_NO_ROOM_TO_PUSH;
    }
    _menuStack[_menuStackIndex++] = _currentMenuContext;
    return MENU_STACK_OK;
}

CMSX::menu_stack_e  CMSX::menuStackPop() // NOLINT(readability-make-member-function-const)
{
    if (_menuStackIndex == 0) {
        return MENU_STACK_NOTHING_TO_POP;
    }
    _currentMenuContext = _menuStack[--_menuStackIndex];
    return MENU_STACK_OK;
}
