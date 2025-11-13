#pragma once

#include <TimeMicroseconds.h>
#include <array>
#include <cstdint>

class Cockpit;
class DisplayPortBase;
class FlightController;
class OSD;
class ReceiverBase;

// CMS MenuExit special ptr values
enum {
    CMS_NULL              = 0,
    CMS_EXIT              = 1,
    CMS_EXIT_SAVE         = 2,
    CMS_EXIT_SAVE_REBOOT  = 3,
    CMS_POPUP_SAVE        = 4,
    CMS_POPUP_SAVE_REBOOT = 5,
    CMS_POPUP_EXIT_REBOOT = 6
};

class CMS {
public:
    CMS(OSD& osd, const ReceiverBase& receiver, const FlightController& flightController, Cockpit& cockpit);
    void init();
private:
    // CMS is not copyable or moveable
    CMS(const CMS&) = delete;
    CMS& operator=(const CMS&) = delete;
    CMS(CMS&&) = delete;
    CMS& operator=(CMS&&) = delete;
public:
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
        KEY_SAVEMENU,
    };
    enum { MAX_DISPLAY_PORT_COUNT = 4 };
    enum { OSD_MENU_ELEMENT_MASK = 0x001F };

    typedef const void* (*entryFnPtr)(CMS& cms, DisplayPortBase& displayPort, const void* ptr);
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
public:
    struct config_t {
        uint8_t filler;
    };
public:
    const config_t& getConfig() const { return _config; }
    void setConfig(const config_t& config);

    void updateCMS(uint32_t currentTimeUs, uint32_t timeMicrosecondsDelta); //!< CMS Task function, called by Task

    uint32_t handleKey(DisplayPortBase& displayPort, key_e key);
    uint32_t scanKeys(uint32_t currentTimeMs, uint32_t lastCalledMs, uint32_t rcDelayMs);
    void setExternKey(key_e externKey);

    const ctx_t& getCurrentCtx() const { return _currentCtx; }
    void setInMenu(bool inMenu) { _inMenu = inMenu; }
    void setCurrentCtxMenuToNull() { _currentCtx.menu = nullptr; }

    void menuOpen();
    static void traverseGlobalExit(const menu_t* menu);
    static const void* menuChange(CMS& cms, DisplayPortBase& displayPort, const void* ptr);
    static const void* menuExit(CMS& cms, DisplayPortBase& displayPort, const void* ptr);
    void inhibitSaveMenu();
    void addMenuEntry(OSD_Entry& menuEntry, const char* text, uint32_t flags, entryFnPtr fnPtr, void* data);
    void drawMenu(DisplayPortBase& displayPort, uint32_t currentTimeUs);
    static menu_t* getSaveExitMenu();

    DisplayPortBase* displayPortSelectNext();
    bool displayPortSelect(const DisplayPortBase* displayPort);
    static bool pwmIsHigh(uint16_t x) { return x > 1750; }
    static bool pwmIsLow(uint16_t x) { return x < 1250; }
    static bool pwmIsMid(uint16_t x) { return (x > 1250) && (x <1750); }

    Cockpit& getCockpit() { return _cockpit; }
private:
    DisplayPortBase* _displayPort {};
    OSD& _osd;
    const ReceiverBase& _receiver;
    const FlightController& _flightController;
    Cockpit& _cockpit;
    config_t _config {};
    ctx_t _currentCtx {};
    int32_t _rcDelayMs {BUTTON_TIME_MS};
    uint32_t _lastCalledMs {};
    uint32_t _lastHeartbeatTimeMs {};
    uint32_t _deviceCount {0};
    int32_t _currentDeviceIndex {-1};

    key_e _externKey {};
    bool _inMenu {};
    bool _saveMenuInhibited {};
    std::array<DisplayPortBase*, MAX_DISPLAY_PORT_COUNT> _displayPorts {};
public:
    enum { MAX_MENU_STACK_SIZE = 10 };
    std::array<ctx_t, MAX_MENU_STACK_SIZE> _menuStack;
    uint8_t _menuStackIndex {0};
    static constexpr void* NULL_PTR = nullptr;
    static const void* EXIT_PTR;
    static const void* EXIT_SAVE_PTR;
    static const void* EXIT_SAVE_REBOOT_PTR;
    static const void* POPUP_SAVE_PTR;
    static const void* POPUP_SAVE_REBOOT_PTR;
    static const void* POPUP_EXIT_REBOOT_PTR;
};

namespace CMSX {
    // Menus
    extern CMS::menu_t menuMain;
    extern CMS::menu_t menuSetPopup;
    extern CMS::menu_t menuImu;
    extern CMS::menu_t menuFeatures;
    extern CMS::menu_t menuOsd;
    extern CMS::menu_t menuFirmware;
    extern CMS::menu_t menuMisc;
    extern CMS::menu_t menuBlackbox;
    extern CMS::menu_t menuPower;
    extern CMS::menu_t menuFailsafe;
} // EMD namespace
