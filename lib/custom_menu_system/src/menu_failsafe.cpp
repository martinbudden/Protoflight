#include "cms.h"
#include "cms_types.h"
#include "cockpit.h"
#include "lookup_tables.h"


static failsafe_config_t failsafe_config {};

// cppcheck-suppress constParameterCallback
static const void* menu_failsafeOnEnter(CMSX& cmsx, cms_context_t& ctx)
{
    (void)cmsx;
    failsafe_config = ctx.cockpit.get_failsafe_config();
    return nullptr;
}

static const void* menu_failsafeOnExit(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    (void)cmsx;
    ctx.cockpit.set_failsafe_config(failsafe_config);
    return nullptr;
}


enum { PWM_MIN = 1000, PWM_MAX = 2000 };

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static_assert(static_cast<int>(LOOKUP_TABLES::FAILSAFE_PROCEDURE_COUNT) == static_cast<int>(Cockpit::FAILSAFE_PROCEDURE_COUNT));
static auto entryFailsafeProcedure   = osd_table_t  { &failsafe_config.procedure, LOOKUP_TABLES::FAILSAFE_PROCEDURE_COUNT - 1, &LOOKUP_TABLES::failsafeProcedureNames[0] };
static auto entryFailsafeDelay       = osd_uint8_t  { &failsafe_config.delay_deciseconds, 0, 200, 1 };
static auto entryFailsafeLandingTime = osd_uint8_t  { &failsafe_config.landing_time_seconds, 0, 200, 1 };
static auto entryFailsafeThrottle    = osd_uint16_t { &failsafe_config.throttle_pwm, PWM_MIN, PWM_MAX, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

// NOLINTBEGIN(hicpp-signed-bitwise)
static const std::array<CMSX::OSD_Entry, 7> menu_failsafeEntries
{{
    { "-- FAILSAFE --", OME_LABEL, nullptr, nullptr},

    { "PROCEDURE",        OME_TABLE  | OME_REBOOT_REQUIRED, nullptr, &entryFailsafeProcedure },
    { "GUARD TIME",       OME_UINT8  | OME_REBOOT_REQUIRED, nullptr, &entryFailsafeDelay },
    { "LANDING_TIME",     OME_UINT8  | OME_REBOOT_REQUIRED, nullptr, &entryFailsafeLandingTime },
    { "STAGE 2 THROTTLE", OME_UINT16 | OME_REBOOT_REQUIRED, nullptr, &entryFailsafeThrottle },
    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};
// NOLINTEND(hicpp-signed-bitwise)

CMSX::menu_t CMSX::menu_failsafe = {
    .on_enter = menu_failsafeOnEnter,
    .on_exit = menu_failsafeOnExit,
    .on_display_update = nullptr,
    .entries = &menu_failsafeEntries[0]
};
