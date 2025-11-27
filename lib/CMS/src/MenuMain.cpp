#include "CMSX.h"
#include "CMS_Types.h"


CMSX::menu_t CMSX::menuBlackbox {};
CMSX::menu_t CMSX::menuPower {};

static const std::array<CMSX::OSD_Entry, 6> menuFeaturesEntries
{{
    {"--- FEATURES ---", OME_LABEL, nullptr, nullptr},

    {"BLACKBOX", OME_SUBMENU, &CMSX::menuChange, &CMSX::menuBlackbox},
    {"POWER",    OME_SUBMENU, &CMSX::menuChange, &CMSX::menuPower},
    {"FAILSAFE", OME_SUBMENU, &CMSX::menuChange, &CMSX::menuFailsafe},

    {"BACK",  OME_BACK, nullptr, nullptr},
    {nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuFeatures = {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuFeaturesEntries[0],
};

static const void* saveExitMenu(CMSX& cmsx, DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::menu_t* menu)
{
    CMSX::menuChange(cmsx, displayPort, cmsx.getSaveExitMenu());
    return nullptr;
}

static const void* menuSetupPopupOnDisplayUpdate(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* selected)
{
    (void)cmsx;

    return nullptr;
}

std::array<CMSX::OSD_Entry, CMSX::SETUP_POPUP_MAX_ENTRIES> CMSX::menuSetupPopupEntries;

CMSX::menu_t CMSX::menuSetupPopup = {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = &menuSetupPopupOnDisplayUpdate,
    .entries = &menuSetupPopupEntries[0],
};

static const void* mainMenuOnEnter(CMSX& cmsx, DisplayPortBase& displayPort)
{
    if (cmsx.setupPopupMenuBuild()) {
        // If setup issues were found then switch to the dynamically constructed menu
        CMSX::menuChange(cmsx, displayPort, &CMSX::menuSetupPopup);
    }
    return nullptr;
}

#if defined(USE_OSD)
static const std::array<CMS::OSD_Entry, 8> menuMainEntries
#else
static const std::array<CMSX::OSD_Entry, 7> menuMainEntries
#endif
{{
    {"-- MAIN --",  OME_LABEL, nullptr, nullptr},

    {"PROFILE",     OME_SUBMENU, &CMSX::menuChange, &CMSX::menuProfile},
    {"FEATURES",    OME_SUBMENU, &CMSX::menuChange, &CMSX::menuFeatures},
#if defined(USE_OSD)
    {"OSD",         OME_SUBMENU, &CMS::menuChange, &CMSX::menuOsd},
#endif
    {"FC&FIRMWARE", OME_SUBMENU, &CMSX::menuChange, &CMSX::menuFirmware},
    {"MISC",        OME_SUBMENU, &CMSX::menuChange, &CMSX::menuMisc},
    {"SAVE/EXIT",   OME_FUNCTION_CALL, &saveExitMenu, nullptr},
    {nullptr, OME_END, nullptr, nullptr},
}};

CMSX::menu_t CMSX::menuMain {
    .onEnter = mainMenuOnEnter,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuMainEntries[0]
};
