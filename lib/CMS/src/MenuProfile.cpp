#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "IMU_Filters.h"
#include "FlightController.h"

//
// Filters
//
static IMU_Filters::config_t imuFiltersConfig {};

static const void* menuFiltersOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* entry)
{
    imuFiltersConfig = cmsx.getCMS().getIMU_Filters().getConfig();
    return nullptr;
}

static const void* menuFiltersOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* entry)
{
    cmsx.getCMS().getIMU_Filters().setConfig(imuFiltersConfig);
    return nullptr;
}

static auto entryGyroLPF1 = OSD_UINT16_t { &imuFiltersConfig.gyro_lpf1_hz, 0, IMU_Filters::GYRO_LPF_MAX_HZ, 1 };
static auto entryGyroLPF2 = OSD_UINT16_t { &imuFiltersConfig.gyro_lpf2_hz, 0, IMU_Filters::GYRO_LPF_MAX_HZ, 1 };
static auto entryGyroNF1  = OSD_UINT16_t { &imuFiltersConfig.gyro_notch1_hz, 0, 500, 1 };
static auto entryGyroNF1C = OSD_UINT16_t { &imuFiltersConfig.gyro_notch1_cutoff, 0, 500, 1 };
static auto entryGyroNF2  = OSD_UINT16_t { &imuFiltersConfig.gyro_notch2_hz, 0, 500, 1 };
static auto entryGyroNF2C = OSD_UINT16_t { &imuFiltersConfig.gyro_notch2_cutoff, 0, 500, 1 };

static const std::array<CMSX::OSD_Entry, 9> menuFiltersEntries =
{{
    { "-- FILTERS --", OME_Label, nullptr, nullptr },

    { "GYRO LPF1",  OME_UINT16 | SLIDER_GYRO, nullptr, &entryGyroLPF1 },
    { "GYRO LPF2",  OME_UINT16 | SLIDER_GYRO, nullptr, &entryGyroLPF2 },
    { "GYRO NF1",   OME_UINT16, nullptr, &entryGyroNF1 },
    { "GYRO NF1C",  OME_UINT16, nullptr, &entryGyroNF1C },
    { "GYRO NF2",   OME_UINT16, nullptr, &entryGyroNF2 },
    { "GYRO NF2C",  OME_UINT16, nullptr, &entryGyroNF2C },

    {"BACK",        OME_Back, nullptr, nullptr},
    {nullptr,       OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuFilters = {
    .onEnter = menuFiltersOnEnter,
    .onExit = menuFiltersOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuFiltersEntries[0]
};

//
// PIDs
//
static std::array<char, 2> pidProfileIndexString = { '1', '\0' };
static std::array<FlightController::PIDF_uint16_t, 3> pids {};

static const void* menuPID_onEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* entry)
{
    pidProfileIndexString[0] = '1' + cmsx.getCMS().getCockpit().getCurrentPidProfileIndex();
    FlightController& flightController = cmsx.getCMS().getCockpit().getFlightController();
    pids[FlightController::ROLL_RATE_DPS] = flightController.getPID_Constants(FlightController::ROLL_RATE_DPS);
    pids[FlightController::PITCH_RATE_DPS] = flightController.getPID_Constants(FlightController::PITCH_RATE_DPS);
    pids[FlightController::YAW_RATE_DPS] = flightController.getPID_Constants(FlightController::YAW_RATE_DPS);
    return nullptr;
}

static const void* menuPID_onExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* entry)
{
    FlightController& flightController = cmsx.getCMS().getCockpit().getFlightController();
    flightController.setPID_Constants(FlightController::ROLL_RATE_DPS, pids[FlightController::ROLL_RATE_DPS]);
    flightController.setPID_Constants(FlightController::PITCH_RATE_DPS, pids[FlightController::PITCH_RATE_DPS]);
    flightController.setPID_Constants(FlightController::YAW_RATE_DPS, pids[FlightController::YAW_RATE_DPS]);
    return nullptr;
}

static auto entryRollPID_P  = OSD_UINT16_t { &pids[0].kp, 0, 200, 1 };
static auto entryRollPID_I  = OSD_UINT16_t { &pids[0].ki, 0, 200, 1 };
static auto entryRollPID_D  = OSD_UINT16_t { &pids[0].kd, 0, 200, 1 };
static auto entryRollPID_K  = OSD_UINT16_t { &pids[0].kk, 0, 200, 1 };
static auto entryPitchPID_P = OSD_UINT16_t { &pids[1].kp, 0, 200, 1 };
static auto entryPitchPID_I = OSD_UINT16_t { &pids[1].ki, 0, 200, 1 };
static auto entryPitchPID_D = OSD_UINT16_t { &pids[1].kd, 0, 200, 1 };
static auto entryPitchPID_K = OSD_UINT16_t { &pids[1].kk, 0, 200, 1 };
static auto entryYawPID_P   = OSD_UINT16_t { &pids[2].kp, 0, 200, 1 };
static auto entryYawPID_I   = OSD_UINT16_t { &pids[2].ki, 0, 200, 1 };
static auto entryYawPID_D   = OSD_UINT16_t { &pids[2].kd, 0, 200, 1 };
static auto entryYawPID_K   = OSD_UINT16_t { &pids[2].kk, 0, 200, 1 };

static const std::array<CMSX::OSD_Entry, 15> menuPidEntries =
{{
    { "-- PID --", OME_Label, nullptr, &pidProfileIndexString[0] },

    { "ROLL  P", OME_UINT16 | SLIDER_RP, nullptr, &entryRollPID_P },
    { "ROLL  I", OME_UINT16 | SLIDER_RP, nullptr, &entryRollPID_I },
    { "ROLL  D", OME_UINT16 | SLIDER_RP, nullptr, &entryRollPID_D },
    { "ROLL  K", OME_UINT16 | SLIDER_RP, nullptr, &entryRollPID_K },

    { "PITCH P", OME_UINT16 | SLIDER_RP, nullptr, &entryPitchPID_P },
    { "PITCH I", OME_UINT16 | SLIDER_RP, nullptr, &entryPitchPID_I },
    { "PITCH D", OME_UINT16 | SLIDER_RP, nullptr, &entryPitchPID_D },
    { "PITCH K", OME_UINT16 | SLIDER_RP, nullptr, &entryPitchPID_K },

    { "YAW   P", OME_UINT16 | SLIDER_RPY, nullptr, &entryYawPID_P },
    { "YAW   I", OME_UINT16 | SLIDER_RPY, nullptr, &entryYawPID_I },
    { "YAW   D", OME_UINT16 | SLIDER_RPY, nullptr, &entryYawPID_D },
    { "YAW   K", OME_UINT16 | SLIDER_RPY, nullptr, &entryYawPID_K },

    { "BACK", OME_Back, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuPID = {
    .onEnter = menuPID_onEnter,
    .onExit = menuPID_onExit,
    .onDisplayUpdate = nullptr,
    .entries = nullptr
};

static const void* pidProfileIndexOnChange(CMSX& cmsx, DisplayPortBase& displayPort, const CMSX::menu_t* menu)
{
    (void)cmsx;
    (void)displayPort;
    (void)menu;
    return nullptr;
}

static std::array<const char * const, 4> pidProfileNames { "1", "2", "3", "4" };

//
// Rates
//

static const void* rateProfileIndexOnChange(CMSX& cmsx, DisplayPortBase& displayPort, const CMSX::menu_t* menu)
{
    (void)cmsx;
    (void)displayPort;
    (void)menu;
    return nullptr;
}

CMSX::menu_t CMSX::menuRates = {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = nullptr
};

static std::array<const char * const, 4> rateProfileNames { "1", "2", "3", "4" };

//
// Profile Menu
//

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryPID_Profile = OSD_TABLE_t  { nullptr, 3, &pidProfileNames[0] };
static auto entryRateProfile = OSD_TABLE_t  { nullptr, 3, &rateProfileNames[0] };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 8> menuProfileEntries =
{{
    {"-- PROFILE --",  OME_Label, nullptr, nullptr},

    {"PID PROF",    OME_TABLE, &pidProfileIndexOnChange, &entryPID_Profile},
    {"PID",         OME_Submenu, &CMSX::menuChange, &CMSX::menuPID},

    {"RATE PROF",   OME_TABLE, &rateProfileIndexOnChange, &entryRateProfile},
    {"RATES",       OME_Submenu, &CMSX::menuChange, &CMSX::menuRates},

    {"FILTERS",     OME_Submenu,&CMSX::menuChange, &CMSX::menuFilters},

    {"BACK",        OME_Back, nullptr, nullptr},
    {nullptr,       OME_END, nullptr, nullptr}
}};

static const void* menuProfileOnEnter(CMSX& cmsx, DisplayPortBase& displayPort, const CMSX::OSD_Entry* entry)
{
    (void)entry;
    CMSX::menuChange(cmsx, displayPort, &CMSX::menuSetPopup);
    return nullptr;
}

static const void* menuProfileOnExit(CMSX& cmsx, DisplayPortBase& displayPort, const CMSX::OSD_Entry* entry)
{
    (void)entry;
    CMSX::menuChange(cmsx, displayPort, &CMSX::menuSetPopup);
    return nullptr;
}

CMSX::menu_t CMSX::menuProfile = {
    .onEnter = menuProfileOnEnter,
    .onExit = menuProfileOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuProfileEntries[0]
};
