#include "CMS.h"
#include "CMS_Types.h"


static const std::array<CMS::OSD_Entry, 6> menuFeaturesEntries =
{{
    {"--- FEATURES ---", OME_Label, nullptr, nullptr},

    {"BLACKBOX", OME_Submenu, &CMS::menuChange, &CMSX::menuBlackbox},
    {"POWER",    OME_Submenu, &CMS::menuChange, &CMSX::menuPower},
    {"FAILSAFE", OME_Submenu, &CMS::menuChange, &CMSX::menuFailsafe},
    {"BACK",     OME_Back, nullptr, nullptr},
    {nullptr, OME_END, nullptr, nullptr}
}};

CMS::menu_t CMSX::menuFeatures = {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuFeaturesEntries[0],
};

static const void* cmsx_SaveExitMenu(CMS& cms, DisplayPortBase& displayPort, const void* ptr)
{
    (void)displayPort;
    (void)ptr;

    CMS::menuChange(cms, displayPort, CMS::getSaveExitMenu());

    return nullptr;
}

#if defined(USE_OSD)
static const std::array<CMS::OSD_Entry, 8> menuMainEntries =
#else
static const std::array<CMS::OSD_Entry, 7> menuMainEntries =
#endif
{{
    {"-- MAIN --",  OME_Label, nullptr, nullptr},

    {"PROFILE",     OME_Submenu,  &CMS::menuChange, &CMSX::menuImu},
    {"FEATURES",    OME_Submenu,  &CMS::menuChange, &CMSX::menuFeatures},
#if defined(USE_OSD)
    {"OSD",         OME_Submenu,  &CMS::menuChange, &CMSX::menuOsd},
#endif
    {"FC&FIRMWARE", OME_Submenu,  &CMS::menuChange, &CMSX::menuFirmware},
    {"MISC",        OME_Submenu,  &CMS::menuChange, &CMSX::menuMisc},
    {"SAVE/EXIT",   OME_Funcall,  &cmsx_SaveExitMenu, nullptr},
    {nullptr, OME_END, nullptr, nullptr},
}};

static const void* mainMenuOnEnter(CMS& cms, DisplayPortBase& displayPort, const CMS::OSD_Entry* self)
{
    (void)self;
#if false
    if (setupPopupMenuBuild()) {
        // If setup issues were found then switch to the dynamically constructed menu
        CMS::menuChange(displayPort, &CMSX::menuSetPopup);
    }
#else
#endif
    CMS::menuChange(cms, displayPort, &CMSX::menuSetPopup);
    return nullptr;
}

CMS::menu_t CMSX::menuMain = {
    .onEnter = mainMenuOnEnter,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuMainEntries[0],
};

CMS::menu_t CMSX::menuSetPopup {};
CMS::menu_t CMSX::menuImu {};
CMS::menu_t CMSX::menuOsd {};
CMS::menu_t CMSX::menuFirmware {};
CMS::menu_t CMSX::menuMisc {};
CMS::menu_t CMSX::menuBlackbox {};
CMS::menu_t CMSX::menuPower {};
