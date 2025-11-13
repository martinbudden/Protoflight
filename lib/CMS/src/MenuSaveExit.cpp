#include "CMS.h"
#include "CMS_Types.h"


// NOLINT_BEGIN(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr)
static const std::array<CMS::OSD_Entry, 6> cmsx_menuSaveExitEntries =
{{
    { "-- SAVE/EXIT --", OME_Label, nullptr, nullptr},
    { "EXIT",            OME_OSD_Exit, &CMS::menuExit, CMS::EXIT_PTR},
    { "SAVE&EXIT",       OME_OSD_Exit, &CMS::menuExit, CMS::POPUP_SAVE_PTR},
    { "SAVE&REBOOT",     OME_OSD_Exit, &CMS::menuExit, CMS::POPUP_SAVE_REBOOT_PTR},
    { "BACK", OME_Back, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

static CMS::menu_t cmsx_menuSaveExit = {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &cmsx_menuSaveExitEntries[0]
};

#if false
static const std::array<CMS::OSD_Entry, 5> cmsx_menuSaveExitRebootEntries =
{
    { "-- SAVE/EXIT (REBOOT REQD)", OME_Label, nullptr, nullptr},
    { "EXIT&REBOOT", OME_OSD_Exit, CMS::menuExit, CMS::POPUP_EXITREBOOT_PTR},
    { "SAVE&REBOOT", OME_OSD_Exit, CMS::menuExit, CMS::POPUP_SAVEREBOOT_PTR},
    { "BACK", OME_Back, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
};

static CMS::menu_t cmsx_menuSaveExitReboot = {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = cmsx_menuSaveExitRebootEntries
};
#endif
// NOLINT_BEGIN(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr)

CMS::menu_t* CMS::getSaveExitMenu()
{
#if false
    if (getRebootRequired()) {
        return &cmsx_menuSaveExitReboot;
    } else {
        return &cmsx_menuSaveExit;
    }
#endif
    return &cmsx_menuSaveExit;
}
