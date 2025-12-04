#include "CMS.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "LookupTables.h"


static Cockpit::failsafe_config_t failsafeConfig {};

static const void* menuFailsafeOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    failsafeConfig = cmsx.getCMS().getCockpit().getFailsafeConfig();
    return nullptr;
}

static const void* menuFailsafeOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    cmsx.getCMS().getCockpit().setFailsafeConfig(failsafeConfig);
    return nullptr;
}


enum { PWM_MIN = 1000, PWM_MAX = 2000 };

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static_assert(static_cast<int>(LOOKUP_TABLES::FAILSAFE_PROCEDURE_COUNT) == static_cast<int>(Cockpit::FAILSAFE_PROCEDURE_COUNT));
static auto entryFailsafeProcedure   = OSD_TABLE_t  { &failsafeConfig.procedure, LOOKUP_TABLES::FAILSAFE_PROCEDURE_COUNT - 1, &LOOKUP_TABLES::failsafeProcedureNames[0] };
static auto entryFailsafeDelay       = OSD_UINT8_t  { &failsafeConfig.delay_deciseconds, 0, 200, 1 };
static auto entryFailsafeLandingTime = OSD_UINT8_t  { &failsafeConfig.landing_time_seconds, 0, 200, 1 };
static auto entryFailsafeThrottle    = OSD_UINT16_t { &failsafeConfig.throttle_pwm, PWM_MIN, PWM_MAX, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

// NOLINTBEGIN(hicpp-signed-bitwise)
static const std::array<CMSX::OSD_Entry, 7> menuFailsafeEntries
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

CMSX::menu_t CMSX::menuFailsafe = {
    .onEnter = menuFailsafeOnEnter,
    .onExit = menuFailsafeOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuFailsafeEntries[0]
};
