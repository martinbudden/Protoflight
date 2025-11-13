#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "Cockpit.h"


static Cockpit::failsafe_t failsafe {};

static const void* cmsx_Failsafe_onEnter(CMS& cms, DisplayPortBase& displayPort, const CMSX::OSD_Entry* self)
{
    (void)displayPort;
    (void)self;

    failsafe = cms.getCockpit().getFailsafe();

    return nullptr;
}

static const void* cmsx_Failsafe_onExit(CMS& cms, DisplayPortBase& displayPort, const CMSX::OSD_Entry* self)
{
    (void)displayPort;
    (void)self;

    cms.getCockpit().setFailsafe(failsafe);

    return nullptr;
}

std::array<const char * const,Cockpit::FAILSAFE_PROCEDURE_COUNT> failsafeProcedureNames = {
    "AUTO-LAND",
    "DROP",
    "GPS-RESCUE",
};

static constexpr int32_t MILLIS_PER_TENTH_SECOND = 100;
enum { PWM_PULSE_MIN = 750 };
enum { PWM_PULSE_MAX = 2250 };

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
auto entry0 = OSD_TAB_t    { &failsafe.procedure, Cockpit::FAILSAFE_PROCEDURE_COUNT - 1, &failsafeProcedureNames[0] };
auto entry1 = OSD_FLOAT_t  { &failsafe.delay, Cockpit::PERIOD_RX_DATA_RECOVERY_MS / MILLIS_PER_TENTH_SECOND, 200, 1, 100 };
auto entry2 = OSD_FLOAT_t  { &failsafe.landing_time, 0, 200, 1, 100 };
auto entry3 = OSD_UINT16_t { &failsafe.throttle, PWM_PULSE_MIN, PWM_PULSE_MAX, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

// NOLINTBEGIN(hicpp-signed-bitwise)
static const std::array<CMSX::OSD_Entry, 7> cmsx_menuFailsafeEntries =
{{
    { "-- FAILSAFE --", OME_Label, nullptr, nullptr},

    { "PROCEDURE",        OME_TAB    | REBOOT_REQUIRED, nullptr, &entry0 },
    { "GUARD TIME",       OME_FLOAT  | REBOOT_REQUIRED, nullptr, &entry1 },
    { "LANDING_TIME",     OME_FLOAT  | REBOOT_REQUIRED, nullptr, &entry2 },
    { "STAGE 2 THROTTLE", OME_UINT16 | REBOOT_REQUIRED, nullptr, &entry3 },
    { "BACK", OME_Back, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};
// NOLINTEND(hicpp-signed-bitwise)

CMSX::menu_t CMSX::menuFailsafe = {
    .onEnter = cmsx_Failsafe_onEnter,
    .onExit = cmsx_Failsafe_onExit,
    .onDisplayUpdate = nullptr,
    .entries = &cmsx_menuFailsafeEntries[0]
};
