#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "Cockpit.h"


static Cockpit::failsafe_t failsafe {};

static const void* menuFailsafeOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    failsafe = cmsx.getCMS().getCockpit().getFailsafe();
    return nullptr;
}

static const void* menuFailsafeOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    cmsx.getCMS().getCockpit().setFailsafe(failsafe);
    return nullptr;
}

static std::array<const char * const, Cockpit::FAILSAFE_PROCEDURE_COUNT> failsafeProcedureNames = {
    "AUTO-LAND",
    "DROP",
    "GPS-RESCUE",
};

static constexpr int32_t MILLIS_PER_TENTH_SECOND = 100;
enum { PWM_PULSE_MIN = 750 };
enum { PWM_PULSE_MAX = 2250 };

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryFailsafeProcedure   = OSD_TABLE_t  { &failsafe.procedure, Cockpit::FAILSAFE_PROCEDURE_COUNT - 1, &failsafeProcedureNames[0] };
static auto entryFailsafeDelay       = OSD_FLOAT_t  { &failsafe.delay, Cockpit::PERIOD_RX_DATA_RECOVERY_MS / MILLIS_PER_TENTH_SECOND, 200, 1, 100 };
static auto entryFailsafeLandingTime = OSD_FLOAT_t  { &failsafe.landing_time, 0, 200, 1, 100 };
static auto entryFailsafeThrottle    = OSD_UINT16_t { &failsafe.throttle, PWM_PULSE_MIN, PWM_PULSE_MAX, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

// NOLINTBEGIN(hicpp-signed-bitwise)
static const std::array<CMSX::OSD_Entry, 7> menuFailsafeEntries
{{
    { "-- FAILSAFE --", OME_Label, nullptr, nullptr},

    { "PROCEDURE",        OME_TABLE  | OME_REBOOT_REQUIRED, nullptr, &entryFailsafeProcedure },
    { "GUARD TIME",       OME_FLOAT  | OME_REBOOT_REQUIRED, nullptr, &entryFailsafeDelay },
    { "LANDING_TIME",     OME_FLOAT  | OME_REBOOT_REQUIRED, nullptr, &entryFailsafeLandingTime },
    { "STAGE 2 THROTTLE", OME_UINT16 | OME_REBOOT_REQUIRED, nullptr, &entryFailsafeThrottle },
    { "BACK", OME_Back, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};
// NOLINTEND(hicpp-signed-bitwise)

CMSX::menu_t CMSX::menuFailsafe = {
    .onEnter = menuFailsafeOnEnter,
    .onExit = menuFailsafeOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuFailsafeEntries[0]
};
