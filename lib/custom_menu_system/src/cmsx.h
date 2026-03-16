#pragma once

#include "targets.h"

#include <array>
#include <cstdint>

class CMS;
class Cockpit;
class DisplayPortBase;
class FlightController;
class ImuFilters;
class ImuBase;
class MotorMixerBase;
class NonVolatileStorage;
class ReceiverBase;
class RcModes;
class VTX;

struct cms_context_t {
    DisplayPortBase& display_port;
    FlightController& flight_controller;
    MotorMixerBase& motor_mixer;
    Cockpit& cockpit;
    ImuFilters& imu_filters;
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
    typedef const void* (*entryFnPtr)(CMSX& cmsx, cms_context_t& ctx, const menu_t* menu);
    struct OSD_Entry {
        const char* text;
        uint16_t flags;
        entryFnPtr fnPtr;
        const void* data;
    };
    typedef const void* (*menuOnEnterFnPtr)(CMSX& cmsx, cms_context_t& ctx);
    typedef const void* (*menuOnExitFnPtr)(CMSX& cmsx, cms_context_t& ctx, const OSD_Entry* self);
    typedef const void* (*menuOnDisplayUpdateFnPtr)(CMSX& cmsx, cms_context_t& ctx, const OSD_Entry* selected);
    struct menu_t {
        menuOnEnterFnPtr on_enter;
        menuOnExitFnPtr on_exit;
        menuOnDisplayUpdateFnPtr on_display_update;
        const OSD_Entry* entries;
    };
    struct menu_context_t {
        const menu_t* menu; // menu for this context
        uint8_t page;       // page in the menu
        uint8_t cursor_row;  // cursor_row in the page
    };
    struct table_ticker_t {
        uint8_t loop_counter;
        uint8_t state;
    };
public:
    bool is_in_menu() const { return _in_menu; }
    void menu_open(cms_context_t& ctx);
    void draw_menu(cms_context_t& ctx, uint32_t current_time_us);
    bool setup_popup_menu_build(cms_context_t& ctx);
    menu_t* get_save_exit_menu() const;
    static const void* inhibit_save_menu(CMSX& cmsx, cms_context_t& ctx) { cmsx.set_save_menu_inhibited(); (void)ctx; return nullptr; }
    void set_save_menu_inhibited() { _save_menu_inhibited = true; }
    uint16_t handle_key(cms_context_t& ctx, key_e key);
    uint16_t handle_key(cms_context_t& ctx, key_e key, const OSD_Entry* entry, uint16_t& entry_flags);
    void save_config_and_notify(cms_context_t& ctx);

    void set_arming_disabled(cms_context_t& ctx);
    void clear_arming_disabled(cms_context_t& ctx);

    uint8_t get_current_pid_profile_index(cms_context_t& ctx) const;
    void set_current_pid_profile_index(cms_context_t& ctx, uint8_t current_pid_profile_index);
    uint8_t get_current_rate_profile_index(cms_context_t& ctx) const;
    void set_current_rate_profile_index(cms_context_t& ctx, uint8_t current_rate_profile_index);
private:
    void set_reboot_required();
    bool get_reboot_required() const;

    bool row_slider_override(const uint16_t flags);
    bool row_is_skippable(const OSD_Entry* row);

    static void pad_left(char *buf, uint8_t size);
    static void pad_right(char *buf, uint8_t size);
    void pad_to_size(char* buf, uint8_t maxSize) const;
    uint32_t draw_menuItemValue(DisplayPortBase& display_port, uint8_t row, uint8_t maxSize);
    uint32_t draw_menuTableItemValue(DisplayPortBase& display_port, uint8_t row, uint8_t maxSize);
    uint32_t draw_menuTableEntry(DisplayPortBase& display_port, const OSD_Entry* entry, uint8_t row, uint16_t& flags, table_ticker_t& ticker);
    uint32_t draw_menuEntry(cms_context_t& ctx, const OSD_Entry* entry, uint8_t row, uint16_t& flags, table_ticker_t& ticker);

    enum { MAX_MENU_STACK_DEPTH = 10 };
    enum menu_stack_e { MENU_STACK_NOTHING_TO_POP, MENU_STACK_NO_ROOM_TO_PUSH, MENU_STACK_OK };
    void menu_stack_reset();
    menu_stack_e menu_stack_push();
    menu_stack_e menu_stack_pop();

    void traverse_global_exit(const menu_t* menu);
    void page_select(uint8_t newpage);
    void page_next();
    void page_previous();

    static void set_flag(uint16_t& value, uint16_t flag) { value |= flag; }
    static void clear_flag(uint16_t& value, uint16_t flag) { value &= static_cast<uint16_t>(~flag); }

public:
    const void* menu_change(cms_context_t& ctx, const menu_t* menu);
    const void* menu_back(cms_context_t& ctx, const menu_t* menu);
    const void* menu_exit(cms_context_t& ctx, const  menu_t* menu);
// static functions with entryFnPtr signature for use by menu system
    static const void* menu_change(CMSX& cmsx, cms_context_t& ctx, const menu_t* menu) { return cmsx.menu_change(ctx, menu); }
    static const void* menu_back(CMSX& cmsx, cms_context_t& ctx, const menu_t* menu) { return cmsx.menu_back(ctx, menu); }
    static const void* menu_exit(CMSX& cmsx, cms_context_t& ctx, const  menu_t* menu) { return cmsx.menu_exit(ctx, menu); }
    static const void* menu_calibrate_gyro(CMSX& cmsx, cms_context_t& ctx, const  menu_t* menu);
    static const void* menu_calibrate_acc(CMSX& cmsx, cms_context_t& ctx, const  menu_t* menu);
    static const void* menu_calibrate_baro(CMSX& cmsx, cms_context_t& ctx, const  menu_t* menu);
    //static const void* inhibit_save_menu(cms_context_t& ctx) { (void)display_port; cmsx.inhibit_save_menu(); return nullptr; }
    enum { CALIBRATION_STATUS_MAX_LENGTH = 6 };
    static std::array<char, CALIBRATION_STATUS_MAX_LENGTH> GyroCalibrationStatus;
    static std::array<char, CALIBRATION_STATUS_MAX_LENGTH> AccCalibrationStatus;
#if defined(USE_BAROMETER)
    static std::array<char, CALIBRATION_STATUS_MAX_LENGTH> BaroCalibrationStatus;
#endif

private:
    CMS& _cms;
    menu_t& _menu_main;
    menu_context_t _current_menu_context {};
    std::array<menu_context_t, MAX_MENU_STACK_DEPTH> _menu_stack {};
    const OSD_Entry* _page_top {}; // First entry for the current page
    uint32_t _last_polled_us {};
    uint16_t _osd_profile_cursor {};
    uint16_t _profile {0};
    uint8_t _menu_stack_index {};
    uint8_t _max_menu_items {};
    uint8_t _page_count {}; // Number of pages in the current menu
    uint8_t _page_max_row {}; // Max row in the current page
    uint8_t _cursor_row {CURSOR_ROW_NOT_SET};
    uint8_t _left_menu_column {};
    uint8_t _right_menu_column {};
    uint8_t _lines_per_menu_item {}; // normally 1, but may be 2 for narrow screens
    bool _reboot_required {false};
    bool _small_screen {false};
    bool _right_aligned {false};
    bool _save_menu_inhibited {false};
    bool _in_menu {};
    bool _elementEditing {};
    std::array<uint16_t, MAX_ROWS> _entry_flags {};
    std::array<table_ticker_t, MAX_ROWS> _runtimeTableTicker {};
    std::array<char, MENU_DRAW_BUFFER_LEN + 2> _menu_draw_buf; // added space for null terminator
    std::array<char, MENU_TABLE_BUFFER_LEN + 2> _menu_table_buf; //added space for null terminator
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

    static std::array<OSD_Entry, SETUP_POPUP_MAX_ENTRIES> menu_setup_popupEntries;

    // Menus
    static menu_t menu_setup_popup;

    static menu_t menu_main;
        static menu_t menu_profile;
            static menu_t menu_pid_tuning;
            static menu_t menu_simplified_pid_Tuning;
            static menu_t menu_rates;
            static menu_t menu_imu_filters;
            static menu_t menu_pid_filters;
        static menu_t menu_features;
            static menu_t menu_blackbox;
            static menu_t menu_vtx;
            static menu_t menu_power;
            static menu_t menu_failsafe;
#if defined(USE_CMS_OSD)
        static menu_t menuOsd;
#endif
        static menu_t menu_firmware;
            // display info
            static menu_t menu_calibrate;
        static menu_t menu_misc;
            static menu_t menu_rc_preview;
        static menu_t menu_save_exit;
        static menu_t menu_save_exit_reboot;

    static menu_t menu_quick;
        static menu_t menu_rpm_limit;
};
