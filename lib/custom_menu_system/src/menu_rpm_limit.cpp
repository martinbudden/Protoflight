#include "cms_types.h"
#include "cmsx.h"
#include "cockpit.h"
#include "flight_controller.h"

#include <motor_mixer_base.h>

static bool dummy0 {};
static uint16_t dummy1 {};
static uint16_t dummy2 {};


dynamic_idle_controller_config_t dynamicIdleControllerConfig {};

// cppcheck-suppress constParameterCallback
static const void* menu_rpm_limitOnEnter(CMSX& cmsx, cms_context_t& ctx)
{
    (void)cmsx;
    const dynamic_idle_controller_config_t* dynamicIdleControllerConfigPtr = ctx.motor_mixer.get_dynamic_idle_config();
    if (dynamicIdleControllerConfigPtr) {
        dynamicIdleControllerConfig = *dynamicIdleControllerConfigPtr;
    }
    return nullptr;
}

static const void* menu_rpm_limitOnExit(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::osd_entry_t* self)
{
    (void)cmsx;
    ctx.motor_mixer.set_dynamic_idle_controller_config(dynamicIdleControllerConfig);
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entry_rpm_limit      = osd_bool_t   { &dummy0 };
static auto entry_rpm_limitValue = osd_uint16_t { &dummy1, 0, UINT16_MAX, 100};
static auto entryKV             = osd_uint16_t { &dummy2, 0, UINT16_MAX, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::osd_entry_t, 7> menu_rpm_limitEntries
{{
    { "-- RPM LIMIT --", OME_LABEL, nullptr, nullptr },

    { "ACTIVE",  OME_BOOL | OME_REBOOT_REQUIRED, nullptr, &entry_rpm_limit },
    { "MAX RPM", OME_UINT16,                     nullptr, &entry_rpm_limitValue },
    { "KV",      OME_UINT16,                     nullptr, &entryKV },

    { "SAVE&REBOOT", OME_EXIT, CMSX::menu_exit, CMSX::MENU_POPUP_SAVE_REBOOT },
    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menu_rpm_limit = {
    .on_enter = menu_rpm_limitOnEnter,
    .on_exit = menu_rpm_limitOnExit,
    .on_display_update = nullptr,
    .entries = &menu_rpm_limitEntries[0]
};
