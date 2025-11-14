#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "DisplayPortBase.h"


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
        runtimeEntryFlags[ii] = entry->flags;
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
