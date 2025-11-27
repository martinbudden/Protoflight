#include "CMS.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "IMU_Filters.h"

/*
Storage.
*/
struct data_t {
    FlightController::simplified_pid_settings_t pidSettings;
};

static data_t data {};
    uint8_t pidTuningMode;


static const void* menuSimplifiedTuningOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    const FlightController& flightController = cmsx.getCMS().getCockpit().getFlightController();
    pidTuningMode = flightController.getPID_TuningMode();
    data.pidSettings = flightController.getSimplifiedPID_Settings();

    return nullptr;
}

static const void* menuSimplifiedTuningOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    FlightController& flightController = cmsx.getCMS().getCockpit().getFlightController();
    flightController.setPID_TuningMode(static_cast<FlightController::pid_tuning_mode_e>(pidTuningMode));
    flightController.setSimplifiedPID_Settings(data.pidSettings);

    return nullptr;
}

static constexpr std::array<const char * const, 3> lookupTablePID_TuningModes { "STANDARD", "RP", "RPY" };
static constexpr std::array<const char * const, 2> lookupTableOffOn { "OFF", "ON" };


// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryTablePID_TuningMode  = OSD_TABLE_t  { &pidTuningMode,  3 - 1, &lookupTablePID_TuningModes[0] };

static auto entryD_gains        = OSD_UINT16_t { &data.pidSettings.d_gain, 0, 200, 1 };
static auto entryPI_gains       = OSD_UINT16_t { &data.pidSettings.pi_gain, 0, 200, 1 };
static auto entryK_gains        = OSD_UINT16_t { &data.pidSettings.k_gain, 0, 200, 1 };
#if defined(USE_D_MAX)
static auto entryDMax           = OSD_UINT16_t { &data.pidSettings.d_max_gain, 0, 200, 1 };
#endif
static auto entryI_gains        = OSD_UINT16_t { &data.pidSettings.i_gain, 0, 200, 1 };
static auto entryRollPitchRatio = OSD_UINT16_t { &data.pidSettings.roll_pitch_ratio, 0, 200, 1 };
static auto entryPitchPI_gains  = OSD_UINT16_t { &data.pidSettings.pitch_pi_gain, 0, 200, 1 };
static auto entryMultiplier     = OSD_UINT16_t { &data.pidSettings.multiplier, 0, 200, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

#if defined(USE_D_MAX)
static const std::array<CMSX::OSD_Entry, 14> menuSimplifiedTuningEntries
#else
static const std::array<CMSX::OSD_Entry, 13> menuSimplifiedTuningEntries
#endif
{{
    { "-- SIMPLIFIED TUNE --", OME_LABEL, nullptr, nullptr},
    { "PID TUNING",        OME_TABLE,  nullptr, &entryTablePID_TuningMode },

    { "-- BASIC --",       OME_LABEL,  nullptr, nullptr},
    { "D GAINS",           OME_UINT16, nullptr, &entryD_gains },
    { "P&I GAINS",         OME_UINT16, nullptr, &entryPI_gains },
    { "FF GAINS",          OME_UINT16, nullptr, &entryK_gains },

    { "-- EXPERT --",      OME_LABEL,  nullptr, nullptr},
#if defined(USE_D_MAX)
    { "D MAX",             OME_UINT16, nullptr, &entryDMax },
#endif
    { "I GAINS",           OME_UINT16, nullptr, &entryI_gains },

    { "PITCH:ROLL D",      OME_UINT16, nullptr, &entryRollPitchRatio },
    { "PITCH:ROLL P,I&FF", OME_UINT16, nullptr, &entryPitchPI_gains },
    { "MASTER MULT",       OME_UINT16, nullptr, &entryMultiplier },

    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuSimplifiedPID_Tuning = {
    .onEnter = menuSimplifiedTuningOnEnter,
    .onExit = menuSimplifiedTuningOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuSimplifiedTuningEntries[0]
};
