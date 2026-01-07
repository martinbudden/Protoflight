#include "CMSX.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "FlightController.h"

#include <MotorMixerBase.h>

static bool dummy0 {};
static uint16_t dummy1 {};
static uint16_t dummy2 {};


DynamicIdleController::config_t dynamicIdleControllerConfig {};

static const void* menuRPM_limitOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    dynamicIdleControllerConfig = cmsx.getCockpit().getFlightController().getMotorMixer().getDynamicIdleController()->getConfig();
    return nullptr;
}

static const void* menuRPM_limitOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    cmsx.getCockpit().getFlightController().getMotorMixer().setDynamicIdlerControllerConfig(dynamicIdleControllerConfig);
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryRPM_limit      = osd_bool_t   { &dummy0 };
static auto entryRPM_limitValue = osd_uint16_t { &dummy1, 0, UINT16_MAX, 100};
static auto entryKV             = osd_uint16_t { &dummy2, 0, UINT16_MAX, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 7> menuRPM_limitEntries
{{
    { "-- RPM LIMIT --", OME_LABEL, nullptr, nullptr },

    { "ACTIVE",  OME_BOOL | OME_REBOOT_REQUIRED, nullptr, &entryRPM_limit },
    { "MAX RPM", OME_UINT16,                     nullptr, &entryRPM_limitValue },
    { "KV",      OME_UINT16,                     nullptr, &entryKV },

    { "SAVE&REBOOT", OME_EXIT, CMSX::menuExit, CMSX::MENU_POPUP_SAVE_REBOOT },
    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuRPM_limit = {
    .onEnter = menuRPM_limitOnEnter,
    .onExit = menuRPM_limitOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuRPM_limitEntries[0]
};
