#include "CMSX.h"
#include "CMS_Types.h"


static const std::array<CMSX::OSD_Entry, 6> menuSaveExitEntries =
{{
    { "-- SAVE/EXIT --", OME_LABEL, nullptr, nullptr},

    { "EXIT",           OME_EXIT, &CMSX::menuExit, CMSX::MENU_EXIT },
    { "SAVE & EXIT",    OME_EXIT, &CMSX::menuExit, CMSX::MENU_POPUP_SAVE },
    { "SAVE & REBOOT",  OME_EXIT, &CMSX::menuExit, CMSX::MENU_POPUP_SAVE_REBOOT },

    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

static const std::array<CMSX::OSD_Entry, 5> menuSaveExitRebootEntries =
{{
    { "-- SAVE/EXIT (REBOOT REQD)", OME_LABEL, nullptr, nullptr },

    { "EXIT & REBOOT",  OME_EXIT, CMSX::menuExit, CMSX::MENU_POPUP_EXIT_REBOOT },
    { "SAVE & REBOOT",  OME_EXIT, CMSX::menuExit, CMSX::MENU_POPUP_SAVE_REBOOT },

    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuSaveExit {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuSaveExitEntries[0]
};

CMSX::menu_t CMSX::menuSaveExitReboot {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuSaveExitRebootEntries[0]
};
