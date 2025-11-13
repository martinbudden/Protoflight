#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "DisplayPortBase.h"

std::array<CMSX::ctx_t, CMSX::MAX_MENU_STACK_SIZE> CMSX::menuStack;
uint8_t CMSX::menuStackIndex {0};
uint8_t CMSX::maxMenuItems {8};
CMSX::ctx_t CMSX::currentCtx {};

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr)
const void* CMSX::MENU_NULL_PTR = reinterpret_cast<const void*>(CMSX::MENU_NULL);
const void* CMSX::EXIT_PTR = reinterpret_cast<const void*>(CMSX::EXIT);
const void* CMSX::EXIT_SAVE_PTR = reinterpret_cast<const void*>(CMSX::EXIT_SAVE);
const void* CMSX::EXIT_SAVE_REBOOT_PTR = reinterpret_cast<const void*>(CMSX::EXIT_SAVE_REBOOT);
const void* CMSX::POPUP_SAVE_PTR = reinterpret_cast<const void*>(POPUP_SAVE);
const void* CMSX::POPUP_SAVE_REBOOT_PTR = reinterpret_cast<const void*>(CMSX::POPUP_SAVE_REBOOT);
const void* CMSX::POPUP_EXIT_REBOOT_PTR = reinterpret_cast<const void*>(CMSX::POPUP_EXIT_REBOOT);
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

//!!TODO need to add menu_t* menu parameter 
const void* CMSX::menuChange(CMS& cms, DisplayPortBase& displayPort, const void* ptr)
{
    //(void)menu;
    (void)cms;
    (void)displayPort;
    (void)ptr;
    return nullptr;
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

const void* CMSX::menuExit(CMS& cms, DisplayPortBase& displayPort, const void* ptr)
{

    if (ptr == EXIT_SAVE_PTR || ptr == EXIT_SAVE_REBOOT_PTR || ptr == POPUP_SAVE_PTR || ptr == POPUP_SAVE_REBOOT_PTR) {
        traverseGlobalExit(&CMSX::menuMain);
        if (currentCtx.menu->onExit) {
            currentCtx.menu->onExit(cms, displayPort, nullptr); // Forced exit
        }
        if ((ptr == POPUP_SAVE_PTR) || (ptr == POPUP_SAVE_REBOOT_PTR)) {
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

    if ((ptr == EXIT_SAVE_REBOOT_PTR) || (ptr == POPUP_SAVE_REBOOT_PTR) || (ptr == POPUP_EXIT_REBOOT_PTR)) {
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
