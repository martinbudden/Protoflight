#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "DisplayPortBase.h"

int CMSX::menuChainBack {};

std::array<CMSX::ctx_t, CMSX::MAX_MENU_STACK_SIZE> CMSX::menuStack;
uint8_t CMSX::menuStackIndex {0};
uint8_t CMSX::maxMenuItems {8};
CMSX::ctx_t CMSX::currentCtx {};
int8_t CMSX::pageCount {};
const CMSX::OSD_Entry* CMSX::pageTop {};
uint8_t CMSX::pageMaxRow {};
bool CMSX::saveMenuInhibited {false};
std::array<uint8_t, CMSX::MAX_ROWS> CMSX::runtimeEntryFlags {};


// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr)
const CMSX::menu_t* CMSX::MENU_NULL_PTR = reinterpret_cast<const CMSX::menu_t*>(CMSX::MENU_NULL);
const CMSX::menu_t* CMSX::EXIT_PTR = reinterpret_cast<const CMSX::menu_t*>(CMSX::EXIT);
const CMSX::menu_t* CMSX::EXIT_SAVE_PTR = reinterpret_cast<const CMSX::menu_t*>(CMSX::EXIT_SAVE);
const CMSX::menu_t* CMSX::EXIT_SAVE_REBOOT_PTR = reinterpret_cast<const CMSX::menu_t*>(CMSX::EXIT_SAVE_REBOOT);
const CMSX::menu_t* CMSX::POPUP_SAVE_PTR = reinterpret_cast<const CMSX::menu_t*>(POPUP_SAVE);
const CMSX::menu_t* CMSX::POPUP_SAVE_REBOOT_PTR = reinterpret_cast<const CMSX::menu_t*>(CMSX::POPUP_SAVE_REBOOT);
const CMSX::menu_t* CMSX::POPUP_EXIT_REBOOT_PTR = reinterpret_cast<const CMSX::menu_t*>(CMSX::POPUP_EXIT_REBOOT);
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr)

uint8_t CMSX::cursorAbsolute()
{ 
    return currentCtx.cursorRow + currentCtx.page * maxMenuItems;
}

void CMSX::addMenuEntry(CMSX::OSD_Entry& menuEntry, const char* text, uint32_t flags, CMSX::entryFnPtr fnPtr, void* data)
{
    menuEntry.text = text;
    menuEntry.flags = flags;
    menuEntry.fnPtr = fnPtr;
    menuEntry.data = data;
}

const void* CMSX::menuChange(CMS& cms, DisplayPortBase& displayPort, const menu_t* menu)
{
    if (!menu) {
        return nullptr;
    }
    if (menu == currentCtx.menu) {
    } else {
        if (currentCtx.menu && menu != &menuMain) {
            // If we are opening the initial top-level menu, then currentCtx.menu will be NULL and there is nothing to do.
            // Otherwise stack the current menu before moving to the selected menu.
            if (menuStackIndex >= MAX_MENU_STACK_SIZE - 1) {
                // menu stack limit reached - prevent array overflow
                return nullptr;
            }
            menuStack[menuStackIndex++] = currentCtx; // push
        }
        currentCtx.menu = menu;
        currentCtx.cursorRow = 0;

        if (menu->onEnter) {
            const void* result = menu->onEnter(cms, displayPort, nullptr);
            if (result == &menuChainBack) {
                return menuBack(cms, displayPort);
            }
        }

    }
    return nullptr;
}

void CMSX::menuCountPage()
{
    const OSD_Entry *p = currentCtx.menu->entries; 
    while ((p->flags & OSD_MENU_ELEMENT_MASK) != OME_END) {
        ++p;
    }
    pageCount = (p - currentCtx.menu->entries - 1) / maxMenuItems + 1;
}

const void* CMSX::menuBack(CMS& cms, DisplayPortBase& displayPort)
{
    if (currentCtx.menu->onExit) {
        const void* result = currentCtx.menu->onExit(cms, displayPort, pageTop + currentCtx.cursorRow);
        if (result == &menuChainBack) {
            return result;
        }
    }
    saveMenuInhibited = false;
    if (!menuStackIndex) {
        return NULL;
    }
    currentCtx = menuStack[--menuStackIndex]; // pop
    menuCountPage();
    pageSelect(displayPort, currentCtx.page);

    return nullptr;
}

void CMSX::updateMaxRow()
{
    pageMaxRow = 0;
    for (const OSD_Entry *ptr = pageTop; (ptr->flags & OSD_MENU_ELEMENT_MASK) != OME_END; ++ptr) {
        ++pageMaxRow;
    }
    if (pageMaxRow > maxMenuItems) {
        pageMaxRow = maxMenuItems;
    }
    if (pageMaxRow > MAX_ROWS) {
        pageMaxRow = MAX_ROWS;
    }
    --pageMaxRow;
}

void CMSX::pageSelect(DisplayPortBase& displayPort, int8_t newpage)
{
    currentCtx.page = (newpage + pageCount) % pageCount;
    pageTop = &currentCtx.menu->entries[currentCtx.page * maxMenuItems];
    updateMaxRow();

    const OSD_Entry* p = pageTop;
    int ii = 0;
    while (p <= pageTop + pageMaxRow) {
        runtimeEntryFlags[ii] = p->flags;
        ++p;
        ++ii;
    }
    displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
}

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast,hicpp-signed-bitwise,misc-no-recursion)
void CMSX::traverseGlobalExit(const CMSX::menu_t* menu)
{
    for (const CMSX::OSD_Entry* p = menu->entries; (p->flags & OSD_MENU_ELEMENT_MASK) != OME_END ; ++p) {
        if ((p->flags & OSD_MENU_ELEMENT_MASK) == OME_Submenu) {
            traverseGlobalExit(reinterpret_cast<const CMSX::menu_t*>(p->data));
        }
    }

}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-reinterpret-cast,hicpp-signed-bitwise,misc-no-recursion)

const void* CMSX::menuExit(CMS& cms, DisplayPortBase& displayPort, const menu_t* menu)
{
    if (menu == EXIT_SAVE_PTR || menu == EXIT_SAVE_REBOOT_PTR || menu == POPUP_SAVE_PTR || menu == POPUP_SAVE_REBOOT_PTR) {
        traverseGlobalExit(&CMSX::menuMain);
        if (currentCtx.menu->onExit) {
            currentCtx.menu->onExit(cms, displayPort, nullptr); // Forced exit
        }
        if ((menu == POPUP_SAVE_PTR) || (menu == POPUP_SAVE_REBOOT_PTR)) {
            // traverse through the menu stack and call all their onExit functions
            for (int ii = menuStackIndex - 1; ii >= 0; --ii) {
                if (menuStack[static_cast<size_t>(ii)].menu->onExit) {
                    menuStack[static_cast<size_t>(ii)].menu->onExit(cms, displayPort, nullptr);
                }
            }
        }
        //saveConfigAndNotify();
    }

    cms.setInMenu(false);
    displayPort.setBackgroundType(DisplayPortBase::BACKGROUND_TRANSPARENT);
    displayPort.release();
    currentCtx.menu = nullptr;

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
