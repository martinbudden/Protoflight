#include "cms_types.h"
#include "cmsx.h"


static const std::array<CMSX::osd_entry_t, 6> menu_save_exitEntries =
{{
    { "-- SAVE/EXIT --", OME_LABEL, nullptr, nullptr},

    { "EXIT",           OME_EXIT, &CMSX::menu_exit, CMSX::MENU_EXIT },
    { "SAVE & EXIT",    OME_EXIT, &CMSX::menu_exit, CMSX::MENU_POPUP_SAVE },
    { "SAVE & REBOOT",  OME_EXIT, &CMSX::menu_exit, CMSX::MENU_POPUP_SAVE_REBOOT },

    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

static const std::array<CMSX::osd_entry_t, 5> menu_save_exit_rebootEntries =
{{
    { "-- SAVE/EXIT (REBOOT REQD)", OME_LABEL, nullptr, nullptr },

    { "EXIT & REBOOT",  OME_EXIT, CMSX::menu_exit, CMSX::MENU_POPUP_EXIT_REBOOT },
    { "SAVE & REBOOT",  OME_EXIT, CMSX::menu_exit, CMSX::MENU_POPUP_SAVE_REBOOT },

    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menu_save_exit {
    .on_enter = nullptr,
    .on_exit = nullptr,
    .on_display_update = nullptr,
    .entries = &menu_save_exitEntries[0]
};

CMSX::menu_t CMSX::menu_save_exit_reboot {
    .on_enter = nullptr,
    .on_exit = nullptr,
    .on_display_update = nullptr,
    .entries = &menu_save_exit_rebootEntries[0]
};
