#include "cms_types.h"
#include "cmsx.h"


static const std::array<CMSX::osd_entry_t, 3> menu_blackboxEntries
{{
    {"-- BLACKBOX --", OME_LABEL, nullptr, nullptr},

    {"BACK",  OME_BACK, nullptr, nullptr},
    {nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menu_blackbox {
    .on_enter = nullptr,
    .on_exit = nullptr,
    .on_display_update = nullptr,
    .entries = &menu_blackboxEntries[0],
};

static const std::array<CMSX::osd_entry_t, 3> menu_powerEntries
{{
    {"-- POWER --", OME_LABEL, nullptr, nullptr},

    {"BACK",  OME_BACK, nullptr, nullptr},
    {nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menu_power {
    .on_enter = nullptr,
    .on_exit = nullptr,
    .on_display_update = nullptr,
    .entries = &menu_powerEntries[0],
};

static const std::array<CMSX::osd_entry_t, 6> menu_featuresEntries
{{
    {"-- FEATURES --", OME_LABEL, nullptr, nullptr},

    {"BLACKBOX", OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_blackbox},
    {"POWER",    OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_power},
    {"FAILSAFE", OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_failsafe},

    {"BACK",  OME_BACK, nullptr, nullptr},
    {nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menu_features {
    .on_enter = nullptr,
    .on_exit = nullptr,
    .on_display_update = nullptr,
    .entries = &menu_featuresEntries[0],
};

static const void* saveExitMenu(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    CMSX::menu_change(cmsx, ctx, cmsx.get_save_exit_menu());
    return nullptr;
}

static const void* menu_setup_popupOnDisplayUpdate(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::osd_entry_t* selected)
{
    (void)cmsx;
    (void)ctx;

    return nullptr;
}

std::array<CMSX::osd_entry_t, CMSX::SETUP_POPUP_MAX_ENTRIES> CMSX::menu_setup_popupEntries;

CMSX::menu_t CMSX::menu_setup_popup = {
    .on_enter = nullptr,
    .on_exit = nullptr,
    .on_display_update = &menu_setup_popupOnDisplayUpdate,
    .entries = &menu_setup_popupEntries[0],
};

static const void* main_menuOnEnter(CMSX& cmsx, cms_context_t& ctx)
{
    if (cmsx.setup_popup_menu_build(ctx)) {
        // If setup issues were found then switch to the dynamically constructed menu
        CMSX::menu_change(cmsx, ctx, &CMSX::menu_setup_popup);
    }
    return nullptr;
}

#if defined(USE_CMS_OSD)
enum { MENU_MAIN_ENTRY_COUNT = 8 };
#else
enum { MENU_MAIN_ENTRY_COUNT = 7 };
#endif
static const std::array<CMSX::osd_entry_t, MENU_MAIN_ENTRY_COUNT> menu_mainEntries
{{
    {"-- MAIN --",  OME_LABEL, nullptr, nullptr},

    {"PROFILE",     OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_profile},
    {"FEATURES",    OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_features},
#if defined(USE_CMS_OSD)
    {"OSD",         OME_SUBMENU, &CMSX::menu_change, &CMSX::menuOsd},
#endif
    {"FC&FIRMWARE", OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_firmware},
    {"MISC",        OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_misc},
    {"SAVE/EXIT",   OME_FUNCTION_CALL, &saveExitMenu, nullptr},
    {nullptr, OME_END, nullptr, nullptr},
}};

CMSX::menu_t CMSX::menu_main {
    .on_enter = main_menuOnEnter,
    .on_exit = nullptr,
    .on_display_update = nullptr,
    .entries = &menu_mainEntries[0]
};
