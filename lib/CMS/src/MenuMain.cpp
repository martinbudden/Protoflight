#include "CMSX.h"
#include "CMS_Types.h"


static const std::array<CMSX::OSD_Entry, 6> menuFeaturesEntries
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

static const void* cmsx_SaveExitMenu(CMSX& cmsx, DisplayPortBase& displayPort, const CMSX::menu_t* menu)
{
    (void)displayPort;
    (void)menu;

    CMSX::menuChange(cmsx, displayPort, CMSX::getSaveExitMenu());

    return nullptr;
}

#if defined(USE_OSD)
static const std::array<CMS::OSD_Entry, 8> menuMainEntries
#else
static const std::array<CMSX::OSD_Entry, 7> menuMainEntries
#endif
{{
    {"-- MAIN --",  OME_Label, nullptr, nullptr},

    {"PROFILE",     OME_Submenu, &CMSX::menuChange, &CMSX::menuProfile},
    {"FEATURES",    OME_Submenu, &CMSX::menuChange, &CMSX::menuFeatures},
#if defined(USE_OSD)
    {"OSD",         OME_Submenu, &CMS::menuChange, &CMSX::menuOsd},
#endif
    {"FC&FIRMWARE", OME_Submenu, &CMSX::menuChange, &CMSX::menuFirmware},
    {"MISC",        OME_Submenu, &CMSX::menuChange, &CMSX::menuMisc},
    {"SAVE/EXIT",   OME_FunctionCall, &cmsx_SaveExitMenu, nullptr},
    {nullptr, OME_END, nullptr, nullptr},
}};

static const void* mainMenuOnEnter(CMSX& cmsx, DisplayPortBase& displayPort)
{
#if false
    if (setupPopupMenuBuild()) {
        // If setup issues were found then switch to the dynamically constructed menu
        CMS::menuChange(cmsx, displayPort, &CMSX::menuSetupPopup);
    }
#else
#endif
    CMSX::menuChange(cmsx, displayPort, &CMSX::menuSetupPopup);
    return nullptr;
}

CMSX::menu_t CMSX::menuMain = {
    .onEnter = mainMenuOnEnter,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuMainEntries[0]
};

static const void* menuSetupPopupOnDisplayUpdate(CMSX& cmsx, DisplayPortBase& displayPort, const CMSX::OSD_Entry* selected)
{
    (void)cmsx;
    (void)displayPort;
    (void)selected;

    return nullptr;
}

#if defined(USE_BATTERY_CONTINUE)
enum { SETUP_POPUP_MAX_ENTRIES = 2 };   // Increase as new entries are added
#else
enum { SETUP_POPUP_MAX_ENTRIES = 1 };   // Increase as new entries are added
#endif
static std::array<CMSX::OSD_Entry, SETUP_POPUP_MAX_ENTRIES + 3> menuSetupPopupEntries;

CMSX::menu_t CMSX::menuSetupPopup = {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = &menuSetupPopupOnDisplayUpdate,
    .entries = &menuSetupPopupEntries[0],
};

CMSX::menu_t CMSX::menuBlackbox {};
CMSX::menu_t CMSX::menuPower {};
CMSX::menu_t CMSX::menuFirmware {};
CMSX::menu_t CMSX::menuMisc {};
CMSX::menu_t CMSX::menuCalibrate {};
