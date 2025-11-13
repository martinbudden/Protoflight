#pragma once

#include <array>
#include <cstdint>

class CMS;
class DisplayPortBase;


namespace CMSX {
    // MenuExit special pointer values
    enum {
        MENU_NULL         = 0,
        EXIT              = 1,
        EXIT_SAVE         = 2,
        EXIT_SAVE_REBOOT  = 3,
        POPUP_SAVE        = 4,
        POPUP_SAVE_REBOOT = 5,
        POPUP_EXIT_REBOOT = 6
    };
    enum { OSD_MENU_ELEMENT_MASK = 0x001F };
    enum { MAX_ROWS = 31 };

    struct menu_t;
    typedef const void* (*entryFnPtr)(CMS& cms, DisplayPortBase& displayPort, const menu_t* ptr);
    struct OSD_Entry {
        const char* text;
        uint32_t flags;
        entryFnPtr fnPtr;
        const void* data;
    };
    typedef const void* (*menuFnPtr)(CMS& cms, DisplayPortBase& displayPort, const OSD_Entry* self);
    struct menu_t {
        menuFnPtr onEnter;
        menuFnPtr onExit;
        menuFnPtr onDisplayUpdate;
        const OSD_Entry* entries;
    };
    struct ctx_t {
        const menu_t* menu; // menu for this context
        uint8_t page;       // page in the menu
        int8_t cursorRow;   // cursorRow in the page
    };

    uint8_t cursorAbsolute();

    void addMenuEntry(OSD_Entry& menuEntry, const char* text, uint32_t flags, entryFnPtr fnPtr, void* data);
    void traverseGlobalExit(const menu_t* menu);
    void menuCountPage();
    void updateMaxRow();
    void pageSelect(DisplayPortBase& displayPort, int8_t newpage);
    const void* menuChange(CMS& cms, DisplayPortBase& displayPort, const menu_t* ptr);
    const void* menuBack(CMS& cms, DisplayPortBase& displayPort);
    const void* menuExit(CMS& cms, DisplayPortBase& displayPort, const  menu_t* ptr);
    menu_t* getSaveExitMenu();

    // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr)
    extern const menu_t* MENU_NULL_PTR;
    extern const menu_t* EXIT_PTR;
    extern const menu_t* EXIT_SAVE_PTR;
    extern const menu_t* EXIT_SAVE_REBOOT_PTR;
    extern const menu_t* POPUP_SAVE_PTR;
    extern const menu_t* POPUP_SAVE_REBOOT_PTR;
    extern const menu_t* POPUP_EXIT_REBOOT_PTR;
    // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr)

    enum { MAX_MENU_STACK_SIZE = 10 };
    extern std::array<ctx_t, MAX_MENU_STACK_SIZE> menuStack;
    extern uint8_t menuStackIndex;
    extern uint8_t maxMenuItems;
    extern ctx_t currentCtx;
    extern int8_t pageCount;         // Number of pages in the current menu
    extern const OSD_Entry* pageTop; // First entry for the current page
    extern uint8_t pageMaxRow;       // Max row in the current page
    extern bool saveMenuInhibited;
    extern std::array<uint8_t, MAX_ROWS> runtimeEntryFlags;
    // Special return value(s) for function chaining by CMSMenuFuncPtr
    extern int menuChainBack;

    // Menus
    extern menu_t menuSetPopup;

    extern menu_t menuMain;
        extern menu_t menuImu;
            //extern menu_t menuPid;
            //extern menu_t menuRates;
            //extern menu_t menuFilters;
        extern menu_t menuFeatures;
            extern menu_t menuBlackbox;
            extern menu_t menuPower;
            extern menu_t menuFailsafe;
        extern menu_t menuOsd;
        extern menu_t menuFirmware;
        extern menu_t menuMisc;
} // END namespace
