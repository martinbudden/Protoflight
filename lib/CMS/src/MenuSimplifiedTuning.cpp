#include "CMS.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "IMU_Filters.h"

/*
Storage.
*/
struct data_t {
    IMU_Filters::config_t imuFiltersConfig {};
    FlightController::simplified_pid_settings_t pidSettings;
};

static data_t data {};



static const void* menuSimplifiedTuningOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    cmsx.getIMU_Filters().setConfig(data.imuFiltersConfig); // NOLINT(cppcoreguidelines-pro-type-union-access)

    const FlightController& flightController = cmsx.getCMS().getCockpit().getFlightController();
    data.pidSettings = flightController.getSimplifiedPID_settings();

    return nullptr;
}

static const void* menuSimplifiedTuningOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    cmsx.getIMU_Filters().setConfig(data.imuFiltersConfig); // NOLINT(cppcoreguidelines-pro-type-union-access)

    FlightController& flightController = cmsx.getCMS().getCockpit().getFlightController();
    flightController.setSimplifiedPID_settings(data.pidSettings);

    return nullptr;
}

static constexpr std::array<const char * const, 3> lookupTableTuningModePIDs { "OFF", "RP", "RPY" };
static constexpr std::array<const char * const, 2> lookupTableOffOn { "OFF", "ON" };


// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static uint8_t dummyTable {};
static uint16_t dummy16 {};
static auto entryTablePID       = OSD_TABLE_t  { &data.pidSettings.mode,  3 - 1, &lookupTableTuningModePIDs[0] };
static auto entryTableGyro      = OSD_TABLE_t  { &dummyTable,  2 - 1, &lookupTableOffOn[0] };
static auto entryTableDTerm     = OSD_TABLE_t  { &dummyTable,  2 - 1, &lookupTableOffOn[0] };

static auto entryD_gains        = OSD_UINT16_t { &data.pidSettings.d_gain, 0, 200, 1 };
static auto entryPI_gains       = OSD_UINT16_t { &data.pidSettings.pi_gain, 0, 200, 1 };
static auto entryK_gains        = OSD_UINT16_t { &data.pidSettings.k_gain, 0, 200, 1 };
static auto entryDMax           = OSD_UINT16_t { &data.pidSettings.d_max_gain, 0, 200, 1 };
static auto entryI_gains        = OSD_UINT16_t { &data.pidSettings.i_gain, 0, 200, 1 };
static auto entryRollPitchRatio = OSD_UINT16_t { &data.pidSettings.roll_pitch_ratio, 0, 200, 1 };
static auto entryPitchPI_gains  = OSD_UINT16_t { &data.pidSettings.pitch_pi_gain, 0, 200, 1 };
static auto entryMultiplier     = OSD_UINT16_t { &data.pidSettings.multiplier, 0, 200, 1 };
static auto entryGyroFilterMultiplier = OSD_UINT16_t { &dummy16, 10, 200, 1 };
static auto entryDTermFilterMultiplier = OSD_UINT16_t { &data.pidSettings.dterm_filter_multiplier, 10, 200, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

// NOLINTBEGIN(hicpp-signed-bitwise)
#ifdef USE_D_MAX
static const std::array<CMSX::OSD_Entry, 20> menuSimplifiedTuningEntries
#else
static const std::array<CMSX::OSD_Entry, 19> menuSimplifiedTuningEntries
#endif
{{
    { "-- SIMPLIFIED PID --", OME_LABEL, nullptr, nullptr},
    { "PID TUNING",        OME_TABLE,  nullptr, &entryTablePID },

    { "-- BASIC --",       OME_LABEL,  nullptr, nullptr},
    { "D GAINS",           OME_UINT16, nullptr, &entryD_gains },
    { "P&I GAINS",         OME_UINT16, nullptr, &entryPI_gains },
    { "FF GAINS",          OME_UINT16, nullptr, &entryK_gains },

    { "-- EXPERT --",      OME_LABEL,  nullptr, nullptr},
#ifdef USE_D_MAX
    { "D MAX",             OME_uINT16, nullptr, &entryDMax },
#endif
    { "I GAINS",           OME_UINT16, nullptr, &entryI_gains },

    { "PITCH:ROLL D",      OME_UINT16, nullptr, &entryRollPitchRatio },
    { "PITCH:ROLL P,I&FF", OME_UINT16, nullptr, &entryPitchPI_gains },
    { "MASTER MULT",       OME_UINT16, nullptr, &entryMultiplier },

    { "-- SIMPLIFIED FILTER --", OME_LABEL, nullptr, nullptr},
    { "GYRO TUNING",       OME_TABLE, nullptr, &entryTableGyro },
    { "GYRO MULT",         OME_UINT16, nullptr, &entryGyroFilterMultiplier },
    { "DTERM TUNING",      OME_TABLE, nullptr, &entryTableDTerm },
    { "DTERM MULT",        OME_UINT16, nullptr, &entryDTermFilterMultiplier },

    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};
// NOLINTEND(hicpp-signed-bitwise)

CMSX::menu_t CMSX::menuSimplifiedTuning = {
    .onEnter = menuSimplifiedTuningOnEnter,
    .onExit = menuSimplifiedTuningOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuSimplifiedTuningEntries[0]
};
