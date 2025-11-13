#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"


static const std::array<CMSX::OSD_Entry, 6> menuFeaturesEntries =
{{
    {"--- FEATURES ---", OME_Label, nullptr, nullptr},

    {"BLACKBOX", OME_Submenu, &CMSX::menuChange, &CMSX::menuBlackbox},
    {"POWER",    OME_Submenu, &CMSX::menuChange, &CMSX::menuPower},
    {"FAILSAFE", OME_Submenu, &CMSX::menuChange, &CMSX::menuFailsafe},
    {"BACK",     OME_Back, nullptr, nullptr},
    {nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuFeatures = {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuFeaturesEntries[0],
};

static const void* cmsx_SaveExitMenu(CMS& cms, DisplayPortBase& displayPort, const void* ptr)
{
    (void)displayPort;
    (void)ptr;

    CMSX::menuChange(cms, displayPort, CMSX::getSaveExitMenu());

    return nullptr;
}

#if defined(USE_OSD)
static const std::array<CMS::OSD_Entry, 8> menuMainEntries =
#else
static const std::array<CMSX::OSD_Entry, 7> menuMainEntries =
#endif
{{
    {"-- MAIN --",  OME_Label, nullptr, nullptr},

    {"PROFILE",     OME_Submenu,  &CMSX::menuChange, &CMSX::menuImu},
    {"FEATURES",    OME_Submenu,  &CMSX::menuChange, &CMSX::menuFeatures},
#if defined(USE_OSD)
    {"OSD",         OME_Submenu,  &CMS::menuChange, &CMSX::menuOsd},
#endif
    {"FC&FIRMWARE", OME_Submenu,  &CMSX::menuChange, &CMSX::menuFirmware},
    {"MISC",        OME_Submenu,  &CMSX::menuChange, &CMSX::menuMisc},
    {"SAVE/EXIT",   OME_Funcall,  &cmsx_SaveExitMenu, nullptr},
    {nullptr, OME_END, nullptr, nullptr},
}};

static const void* mainMenuOnEnter(CMS& cms, DisplayPortBase& displayPort, const CMSX::OSD_Entry* self)
{
    (void)self;
#if false
    if (setupPopupMenuBuild()) {
        // If setup issues were found then switch to the dynamically constructed menu
        CMS::menuChange(cms, displayPort, &CMSX::menuSetPopup);
    }
#else
#endif
    CMSX::menuChange(cms, displayPort, &CMSX::menuSetPopup);
    return nullptr;
}

CMSX::menu_t CMSX::menuMain = {
    .onEnter = mainMenuOnEnter,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuMainEntries[0],
};

CMSX::menu_t CMSX::menuSetPopup {};
CMSX::menu_t CMSX::menuImu {};
CMSX::menu_t CMSX::menuOsd {};
CMSX::menu_t CMSX::menuFirmware {};
CMSX::menu_t CMSX::menuMisc {};
CMSX::menu_t CMSX::menuBlackbox {};
CMSX::menu_t CMSX::menuPower {};
