#pragma once

#include "Targets.h"

#include <array>
#include <cstdint>

class CMS;
class Cockpit;
class DisplayPortBase;
class FlightController;
class IMU_Filters;
class ImuBase;
class MotorMixerBase;
class NonVolatileStorage;
class ReceiverBase;
class RcModes;
class VTX;

struct cms_parameter_group_t {
    DisplayPortBase& displayPort;
    FlightController& flightController;
    MotorMixerBase& motorMixer;
    Cockpit& cockpit;
    IMU_Filters& imuFilters;
    ImuBase& imu; // needed for IMU calibration
    RcModes& rc_modes;
    ReceiverBase& receiver;
    NonVolatileStorage& nvs;
    VTX* vtx;
};

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
    enum { CURSOR_ROW_NOT_SET = 255 };

    enum { MAX_ROWS = 31 };
    static constexpr uint8_t MENU_DRAW_BUFFER_LEN = 12;
    static constexpr uint8_t MENU_TABLE_BUFFER_LEN = 30;
    enum { NUMBER_FIELD_LEN = 5 };
    enum { DYNAMIC_VALUES_POLLING_INTERVAL_US = 100'000 };
    enum { BUTTON_TIME_MS = 250 };
    enum { BUTTON_PAUSE_MS = 500 };
    enum { LOOKUP_TABLE_TICKER_START_CYCLES = 20,  // Task loops for start/end of ticker (1 second delay)
           LOOKUP_TABLE_TICKER_SCROLL_CYCLES = 3   // Task loops for each scrolling step of the ticker (150ms delay)
    };
    enum { NORMAL_SCREEN_MIN_COLS = 18,      // Less is a small screen
           NORMAL_SCREEN_MAX_COLS = 30      // More is a large screen
    };
#if defined(USE_BATTERY_CONTINUE)
    enum { SETUP_POPUP_MAX_ENTRIES = 5 };   // Increase as new entries are added
#else
    enum { SETUP_POPUP_MAX_ENTRIES = 4 };   // Increase as new entries are added
#endif
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
    typedef const void* (*entryFnPtr)(CMSX& cmsx, cms_parameter_group_t& pg, const menu_t* menu);
    struct OSD_Entry {
        const char* text;
        uint16_t flags;
        entryFnPtr fnPtr;
        const void* data;
    };
    typedef const void* (*menuOnEnterFnPtr)(CMSX& cmsx, cms_parameter_group_t& pg);
    typedef const void* (*menuOnExitFnPtr)(CMSX& cmsx, cms_parameter_group_t& pg, const OSD_Entry* self);
    typedef const void* (*menuOnDisplayUpdateFnPtr)(CMSX& cmsx, cms_parameter_group_t& pg, const OSD_Entry* selected);
    struct menu_t {
        menuOnEnterFnPtr onEnter;
        menuOnExitFnPtr onExit;
        menuOnDisplayUpdateFnPtr onDisplayUpdate;
        const OSD_Entry* entries;
    };
    struct menu_context_t {
        const menu_t* menu; // menu for this context
        uint8_t page;       // page in the menu
        uint8_t cursorRow;  // cursorRow in the page
    };
    struct table_ticker_t {
        uint8_t loopCounter;
        uint8_t state;
    };
public:
    bool isInMenu() const { return _inMenu; }
    void menuOpen(cms_parameter_group_t& pg);
    void drawMenu(cms_parameter_group_t& pg, uint32_t currentTimeUs);
    bool setupPopupMenuBuild(cms_parameter_group_t& pg);
    menu_t* getSaveExitMenu() const;
    static const void* inhibitSaveMenu(CMSX& cmsx, cms_parameter_group_t& pg) { cmsx.setSaveMenuInhibited(); (void)pg; return nullptr; }
    void setSaveMenuInhibited() { _saveMenuInhibited = true; }
    uint16_t handleKey(cms_parameter_group_t& pg, key_e key);
    uint16_t handleKey(cms_parameter_group_t& pg, key_e key, const OSD_Entry* entry, uint16_t& entryFlags);
    void saveConfigAndNotify(cms_parameter_group_t& pg);

    void setArmingDisabled(cms_parameter_group_t& pg);
    void clearArmingDisabled(cms_parameter_group_t& pg);

    uint8_t get_current_pid_profile_index(cms_parameter_group_t& pg) const;
    void set_current_pid_profile_index(cms_parameter_group_t& pg, uint8_t current_pid_profile_index);
    uint8_t get_current_rate_profile_index(cms_parameter_group_t& pg) const;
    void set_current_rate_profile_index(cms_parameter_group_t& pg, uint8_t current_rate_profile_index);
private:
    void setRebootRequired();
    bool getRebootRequired() const;

    bool rowSliderOverride(const uint16_t flags);
    bool rowIsSkippable(const OSD_Entry* row);

    static void padLeft(char *buf, uint8_t size);
    static void padRight(char *buf, uint8_t size);
    void padToSize(char* buf, uint8_t maxSize) const;
    uint32_t drawMenuItemValue(DisplayPortBase& displayPort, uint8_t row, uint8_t maxSize);
    uint32_t drawMenuTableItemValue(DisplayPortBase& displayPort, uint8_t row, uint8_t maxSize);
    uint32_t drawMenuTableEntry(DisplayPortBase& displayPort, const OSD_Entry* entry, uint8_t row, uint16_t& flags, table_ticker_t& ticker);
    uint32_t drawMenuEntry(cms_parameter_group_t& pg, const OSD_Entry* entry, uint8_t row, uint16_t& flags, table_ticker_t& ticker);

    enum { MAX_MENU_STACK_DEPTH = 10 };
    enum menu_stack_e { MENU_STACK_NOTHING_TO_POP, MENU_STACK_NO_ROOM_TO_PUSH, MENU_STACK_OK };
    void menuStackReset();
    menu_stack_e menuStackPush();
    menu_stack_e menuStackPop();

    void traverseGlobalExit(const menu_t* menu);
    void pageSelect(uint8_t newpage);
    void pageNext();
    void pagePrevious();

    static void setFlag(uint16_t& value, uint16_t flag) { value |= flag; }
    static void clearFlag(uint16_t& value, uint16_t flag) { value &= static_cast<uint16_t>(~flag); }

public:
    const void* menuChange(cms_parameter_group_t& pg, const menu_t* menu);
    const void* menuBack(cms_parameter_group_t& pg, const menu_t* menu);
    const void* menuExit(cms_parameter_group_t& pg, const  menu_t* menu);
// static functions with entryFnPtr signature for use by menu system
    static const void* menuChange(CMSX& cmsx, cms_parameter_group_t& pg, const menu_t* menu) { return cmsx.menuChange(pg, menu); }
    static const void* menuBack(CMSX& cmsx, cms_parameter_group_t& pg, const menu_t* menu) { return cmsx.menuBack(pg, menu); }
    static const void* menuExit(CMSX& cmsx, cms_parameter_group_t& pg, const  menu_t* menu) { return cmsx.menuExit(pg, menu); }
    static const void* menuCalibrateGyro(CMSX& cmsx, cms_parameter_group_t& pg, const  menu_t* menu);
    static const void* menuCalibrateAcc(CMSX& cmsx, cms_parameter_group_t& pg, const  menu_t* menu);
    static const void* menuCalibrateBaro(CMSX& cmsx, cms_parameter_group_t& pg, const  menu_t* menu);
    //static const void* inhibitSaveMenu(cms_parameter_group_t& pg) { (void)displayPort; cmsx.inhibitSaveMenu(); return nullptr; }
    enum { CALIBRATION_STATUS_MAX_LENGTH = 6 };
    static std::array<char, CALIBRATION_STATUS_MAX_LENGTH> GyroCalibrationStatus;
    static std::array<char, CALIBRATION_STATUS_MAX_LENGTH> AccCalibrationStatus;
#if defined(USE_BAROMETER)
    static std::array<char, CALIBRATION_STATUS_MAX_LENGTH> BaroCalibrationStatus;
#endif

private:
    CMS& _cms;
    menu_t& _menuMain;
    menu_context_t _currentMenuContext {};
    std::array<menu_context_t, MAX_MENU_STACK_DEPTH> _menuStack {};
    const OSD_Entry* _pageTop {}; // First entry for the current page
    uint32_t _lastPolledUs {};
    uint16_t _osdProfileCursor {};
    uint16_t _profile {0};
    uint8_t _menuStackIndex {};
    uint8_t _maxMenuItems {};
    uint8_t _pageCount {}; // Number of pages in the current menu
    uint8_t _pageMaxRow {}; // Max row in the current page
    uint8_t _cursorRow {CURSOR_ROW_NOT_SET};
    uint8_t _leftMenuColumn {};
    uint8_t _rightMenuColumn {};
    uint8_t _linesPerMenuItem {}; // normally 1, but may be 2 for narrow screens
    bool _rebootRequired {false};
    bool _smallScreen {false};
    bool _rightAligned {false};
    bool _saveMenuInhibited {false};
    bool _inMenu {};
    bool _elementEditing {};
    std::array<uint16_t, MAX_ROWS> _entryFlags {};
    std::array<table_ticker_t, MAX_ROWS> _runtimeTableTicker {};
    std::array<char, MENU_DRAW_BUFFER_LEN + 2> _menuDrawBuf; // added space for null terminator
    std::array<char, MENU_TABLE_BUFFER_LEN + 2> _menuTableBuf; //added space for null terminator
public:
    // MenuExit special pointer values
    static const menu_t* MENU_NULL_PTR;
    static const menu_t* MENU_EXIT;
    static const menu_t* MENU_EXIT_SAVE;
    static const menu_t* MENU_EXIT_SAVE_REBOOT;
    static const menu_t* MENU_POPUP_SAVE;
    static const menu_t* MENU_POPUP_SAVE_REBOOT;
    static const menu_t* MENU_POPUP_EXIT_REBOOT;
    static const menu_t* MENU_BACK;

    static std::array<OSD_Entry, SETUP_POPUP_MAX_ENTRIES> menuSetupPopupEntries;

    // Menus
    static menu_t menuSetupPopup;

    static menu_t menuMain;
        static menu_t menuProfile;
            static menu_t menuPID_Tuning;
            static menu_t menuSimplifiedPID_Tuning;
            static menu_t menuRates;
            static menu_t menuIMU_Filters;
            static menu_t menuPID_Filters;
        static menu_t menuFeatures;
            static menu_t menuBlackbox;
            static menu_t menuVTX;
            static menu_t menuPower;
            static menu_t menuFailsafe;
#if defined(USE_CMS_OSD)
        static menu_t menuOsd;
#endif
        static menu_t menuFirmware;
            // display info
            static menu_t menuCalibrate;
        static menu_t menuMisc;
            static menu_t menuRcPreview;
        static menu_t menuSaveExit;
        static menu_t menuSaveExitReboot;

    static menu_t menuQuick;
        static menu_t menuRPM_limit;
};
