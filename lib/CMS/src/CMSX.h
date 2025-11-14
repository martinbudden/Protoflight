#pragma once

#include <array>
#include <cstdint>

class CMS;
class DisplayPortBase;


class CMSX {
public:
    CMSX(CMS& cms);
private:
    // CMS is not copyable or moveable
    CMSX(const CMSX&) = delete;
    CMSX& operator=(const CMSX&) = delete;
    CMSX(CMSX&&) = delete;
    CMSX& operator=(CMSX&&) = delete;
public:
    enum { MAX_ROWS = 31 };
    struct menu_t;
    typedef const void* (*entryFnPtr)(CMSX& cmsx, DisplayPortBase& displayPort, const menu_t* menu);
    struct OSD_Entry {
        const char* text;
        uint32_t flags;
        entryFnPtr fnPtr;
        const void* data;
    };
    typedef const void* (*menuFnPtr)(CMSX& cmsx, DisplayPortBase& displayPort, const OSD_Entry* entry);
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
public:
    uint8_t cursorAbsolute() const;
    void setInMenu(bool inMenu);
    bool isInMenu() const;

    void addMenuEntry(OSD_Entry& menuEntry, const char* text, uint32_t flags, entryFnPtr fnPtr, void* data);
    void traverseGlobalExit(const menu_t* menu);
    void menuCountPage();
    void updateMaxRow();
    void pageSelect(DisplayPortBase& displayPort, int8_t newpage);

    CMS& getCMS() { return _cms; }

// static functions with entryFnPtr signature for use by menu system
    static const void* menuChange(CMSX& cmsx, DisplayPortBase& displayPort, const menu_t* menu);
    static const void* menuBack(CMSX& cmsx, DisplayPortBase& displayPort, const menu_t* menu);
    static const void* menuExit(CMSX& cmsx, DisplayPortBase& displayPort, const  menu_t* menu);
    static menu_t* getSaveExitMenu();

// variables public for now
    ctx_t _currentCtx {};
    uint8_t _menuStackIndex {};
private:
    CMS& _cms;
    enum { MAX_MENU_STACK_DEPTH = 10 };
    std::array<ctx_t, MAX_MENU_STACK_DEPTH> _menuStack {};
    const OSD_Entry* _pageTop {}; // First entry for the current page
    
    int _menuChainBack {}; // Special return value for function chaining by menu function pointer
    uint8_t _maxMenuItems {};
    int8_t _pageCount {};         // Number of pages in the current menu
    uint8_t _pageMaxRow {};       // Max row in the current page
    bool _saveMenuInhibited {};
    bool _inMenu {};
    std::array<uint8_t, MAX_ROWS> runtimeEntryFlags {};
public:
    // MenuExit special pointer values
    static const menu_t* MENU_NULL_PTR;
    static const menu_t* EXIT_PTR;
    static const menu_t* EXIT_SAVE_PTR;
    static const menu_t* EXIT_SAVE_REBOOT_PTR;
    static const menu_t* POPUP_SAVE_PTR;
    static const menu_t* POPUP_SAVE_REBOOT_PTR;
    static const menu_t* POPUP_EXIT_REBOOT_PTR;

    // Menus
    static menu_t menuSetPopup;

    static menu_t menuMain;
        static menu_t menuImu;
            //static menu_t menuPid;
            //static menu_t menuRates;
            //static menu_t menuFilters;
        static menu_t menuFeatures;
            static menu_t menuBlackbox;
            static menu_t menuPower;
            static menu_t menuFailsafe;
        static menu_t menuOsd;
        static menu_t menuFirmware;
        static menu_t menuMisc;
}; // END namespace
