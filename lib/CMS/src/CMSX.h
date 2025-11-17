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
    enum { MENU_DRAW_BUFFER_LEN = 12, MENU_TABLE_BUFFER_LEN = 30 };
    enum { NUMBER_FIELD_LEN = 5 };
    enum { CMS_POLL_INTERVAL_US = 100'000 };  // Interval for polling dynamic values
    enum { BUTTON_TIME_MS = 250 };
    enum { BUTTON_PAUSE_MS = 500 };
    enum key_e {
        KEY_NONE,
        KEY_UP,
        KEY_DOWN,
        KEY_LEFT,
        KEY_RIGHT,
        KEY_ESC,
        KEY_MENU,
        KEY_SAVE_MENU,
    };
public:
    struct menu_t;
    typedef const void* (*entryFnPtr)(CMSX& cmsx, DisplayPortBase& displayPort, const menu_t* menu);
    struct OSD_Entry {
        const char* text;
        uint16_t flags;
        entryFnPtr fnPtr;
        const void* data;
    };
    typedef const void* (*menuOnEnterFnPtr)(CMSX& cmsx, DisplayPortBase& displayPort);
    typedef const void* (*menuOnExitFnPtr)(CMSX& cmsx, DisplayPortBase& displayPort, const OSD_Entry* self);
    typedef const void* (*menuOnDisplayUpdateFnPtr)(CMSX& cmsx, DisplayPortBase& displayPort, const OSD_Entry* selected);
    struct menu_t {
        menuOnEnterFnPtr onEnter;
        menuOnExitFnPtr onExit;
        menuOnDisplayUpdateFnPtr onDisplayUpdate;
        const OSD_Entry* entries;
    };
    struct ctx_t {
        const menu_t* menu; // menu for this context
        uint8_t page;       // page in the menu
        uint8_t cursorRow;   // cursorRow in the page
    };
    struct table_ticker_t {
        uint8_t loopCounter;
        uint8_t state;
    };
public:
    uint8_t cursorAbsolute() const;
    void setInMenu(bool inMenu);
    bool isInMenu() const;
    void setRebootRequired();

    bool rowSliderOverride(const uint16_t flags);
    bool rowIsSkippable(const OSD_Entry* row);

    void menuOpen(DisplayPortBase& displayPort);
    const void* menuBack(DisplayPortBase& displayPort);
    void drawMenu(DisplayPortBase& displayPort, uint32_t currentTimeUs);
    uint32_t drawMenuItemValue(DisplayPortBase& displayPort, uint8_t* buf, uint8_t row, uint8_t maxSize);
    uint32_t drawMenuEntry(DisplayPortBase& displayPort, const OSD_Entry* entry, uint8_t row, bool selectedRow, uint8_t index);

    uint16_t handleKey(DisplayPortBase& displayPort, key_e key);

    void addMenuEntry(OSD_Entry& menuEntry, const char* text, uint16_t flags, entryFnPtr fnPtr, void* data);
    void traverseGlobalExit(const menu_t* menu);
    void menuCountPage();
    void updateMaxRow();
    void pageSelect(DisplayPortBase& displayPort, uint8_t newpage);
    void pageNext(DisplayPortBase& displayPort);
    void pagePrevious(DisplayPortBase& displayPort);

    CMS& getCMS() { return _cms; }

// static functions with entryFnPtr signature for use by menu system
    static const void* menuChange(CMSX& cmsx, DisplayPortBase& displayPort, const menu_t* menu);
    static const void* menuBack(CMSX& cmsx, DisplayPortBase& displayPort, const menu_t* menu);
    static const void* menuExit(CMSX& cmsx, DisplayPortBase& displayPort, const  menu_t* menu);
    static menu_t* getSaveExitMenu();

private:
    CMS& _cms;
    enum { MAX_MENU_STACK_DEPTH = 10 };
    ctx_t _currentCtx {};
    uint8_t _menuStackIndex {};
    std::array<ctx_t, MAX_MENU_STACK_DEPTH> _menuStack {};
    const OSD_Entry* _pageTop {}; // First entry for the current page
    uint32_t _lastPolledUs {};
    uint32_t _osdProfileCursor {};
    uint8_t _maxMenuItems {};
    uint8_t _pageCount {}; // Number of pages in the current menu
    uint8_t _pageMaxRow {}; // Max row in the current page
    uint8_t _leftMenuColumn {};
    uint8_t _rightMenuColumn {};
    uint8_t _linesPerMenuItem {};
    bool _smallScreen {false};
    bool _saveMenuInhibited {};
    bool _inMenu {};
    bool _elementEditing {};
    std::array<uint16_t, MAX_ROWS> _runtimeEntryFlags {};
    std::array<table_ticker_t, MAX_ROWS> _runtimeTableTicker {};
    std::array<uint8_t, MENU_DRAW_BUFFER_LEN + 2> _menuDrawBuf; // added space for null terminator
    std::array<uint8_t, MENU_TABLE_BUFFER_LEN + 2> _menuTableBuf; //added space for null terminator
public:
    // MenuExit special pointer values
    static const menu_t* MENU_NULL_PTR;
    static const menu_t* MENU_EXIT;
    static const menu_t* MENU_EXIT_SAVE;
    static const menu_t* MENU_EXIT_SAVE_REBOOT;
    static const menu_t* MENU_POPUP_SAVE;
    static const menu_t* MENU_POPUP_SAVE_REBOOT;
    static const menu_t* MENU_POPUP_EXIT_REBOOT;
    static const menu_t* MENU_CHAIN_BACK;

    // Menus
    static menu_t menuSetupPopup;

    static menu_t menuMain;
        static menu_t menuProfile;
            static menu_t menuPID;
            static menu_t menuRates;
            static menu_t menuFilters;
        static menu_t menuFeatures;
            static menu_t menuBlackbox;
            static menu_t menuPower;
            static menu_t menuFailsafe;
        //static menu_t menuOsd;
        static menu_t menuFirmware;
            // display info
            static menu_t menuCalibrate;
        static menu_t menuMisc;
};
