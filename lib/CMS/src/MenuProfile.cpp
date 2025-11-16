#include "CMS.h"
#include "CMSX.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "IMU_Filters.h"

/*
Storage

Only one menu can be active at one time, so we can save RAM by using a union.
*/
union data_u {
    IMU_Filters::config_t imuFiltersConfig {};
    Cockpit::rates_t rates;
    std::array<FlightController::PIDF_uint16_t, 3> pids;
};

static data_u data {};

//
// Filters
//

static const void* menuFiltersOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* entry)
{
    data.imuFiltersConfig = cmsx.getCMS().getIMU_Filters().getConfig(); // NOLINT(cppcoreguidelines-pro-type-union-access)
    return nullptr;
}

static const void* menuFiltersOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* entry)
{
    cmsx.getCMS().getIMU_Filters().setConfig(data.imuFiltersConfig); // NOLINT(cppcoreguidelines-pro-type-union-access)
    return nullptr;
}

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,fuchsia-statically-constructed-objects)
static auto entryGyroLPF1 = OSD_UINT16_t { &data.imuFiltersConfig.gyro_lpf1_hz, 0, IMU_Filters::GYRO_LPF_MAX_HZ, 1 };
static auto entryGyroLPF2 = OSD_UINT16_t { &data.imuFiltersConfig.gyro_lpf2_hz, 0, IMU_Filters::GYRO_LPF_MAX_HZ, 1 };
static auto entryGyroNF1  = OSD_UINT16_t { &data.imuFiltersConfig.gyro_notch1_hz, 0, 500, 1 };
static auto entryGyroNF1C = OSD_UINT16_t { &data.imuFiltersConfig.gyro_notch1_cutoff, 0, 500, 1 };
static auto entryGyroNF2  = OSD_UINT16_t { &data.imuFiltersConfig.gyro_notch2_hz, 0, 500, 1 };
static auto entryGyroNF2C = OSD_UINT16_t { &data.imuFiltersConfig.gyro_notch2_cutoff, 0, 500, 1 };
// NOLINTEND(cppcoreguidelines-pro-type-union-access,fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 9> menuFiltersEntries
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
static std::array<uint8_t, 2> pidProfileIndexString = { '1', '\0' };

static const void* menuPID_onEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* entry)
{
    pidProfileIndexString[0] = '1' + cmsx.getCMS().getCockpit().getCurrentPidProfileIndex();
    const FlightController& flightController = cmsx.getCMS().getCockpit().getFlightController();
    data.pids[FlightController::ROLL_RATE_DPS] = flightController.getPID_Constants(FlightController::ROLL_RATE_DPS);
    data.pids[FlightController::PITCH_RATE_DPS] = flightController.getPID_Constants(FlightController::PITCH_RATE_DPS);
    data.pids[FlightController::YAW_RATE_DPS] = flightController.getPID_Constants(FlightController::YAW_RATE_DPS);
    return nullptr;
}

static const void* menuPID_onExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* entry)
{
    FlightController& flightController = cmsx.getCMS().getCockpit().getFlightController();
    flightController.setPID_Constants(FlightController::ROLL_RATE_DPS, data.pids[FlightController::ROLL_RATE_DPS]);
    flightController.setPID_Constants(FlightController::PITCH_RATE_DPS, data.pids[FlightController::PITCH_RATE_DPS]);
    flightController.setPID_Constants(FlightController::YAW_RATE_DPS, data.pids[FlightController::YAW_RATE_DPS]);
    return nullptr;
}

static auto entryRollPID_P  = OSD_UINT16_t { &data.pids[0].kp, 0, 200, 1 };
static auto entryRollPID_I  = OSD_UINT16_t { &data.pids[0].ki, 0, 200, 1 };
static auto entryRollPID_D  = OSD_UINT16_t { &data.pids[0].kd, 0, 200, 1 };
static auto entryRollPID_K  = OSD_UINT16_t { &data.pids[0].kk, 0, 200, 1 };
static auto entryPitchPID_P = OSD_UINT16_t { &data.pids[1].kp, 0, 200, 1 };
static auto entryPitchPID_I = OSD_UINT16_t { &data.pids[1].ki, 0, 200, 1 };
static auto entryPitchPID_D = OSD_UINT16_t { &data.pids[1].kd, 0, 200, 1 };
static auto entryPitchPID_K = OSD_UINT16_t { &data.pids[1].kk, 0, 200, 1 };
static auto entryYawPID_P   = OSD_UINT16_t { &data.pids[2].kp, 0, 200, 1 };
static auto entryYawPID_I   = OSD_UINT16_t { &data.pids[2].ki, 0, 200, 1 };
static auto entryYawPID_D   = OSD_UINT16_t { &data.pids[2].kd, 0, 200, 1 };
static auto entryYawPID_K   = OSD_UINT16_t { &data.pids[2].kk, 0, 200, 1 };

static const std::array<CMSX::OSD_Entry, 15> menuPidEntries
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
    .entries = &menuPidEntries[0]
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
static std::array<uint8_t, 2> rateProfileIndexString = { '1', '\0' };

static const void* menuRatesOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* entry)
{
    data.rates = cmsx.getCMS().getCockpit().getRates();
    return nullptr;
}

static const void* menuRatesOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* entry)
{
    cmsx.getCMS().getCockpit().setRates(data.rates);
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static std::array<const char * const, 3> lookupTableThrottleLimitType { "OFF", "SCALE", "CLIP" };

static auto entryRcRatesRoll  = OSD_FLOAT_t { &data.rates.rcRates[FlightController::FD_ROLL], 1, 255, 1, 10 };
static auto entryRcRatesPitch = OSD_FLOAT_t { &data.rates.rcRates[FlightController::FD_PITCH], 1, 255, 1, 10 };
static auto entryRcRatesYaw   = OSD_FLOAT_t { &data.rates.rcRates[FlightController::FD_YAW], 1, 255, 1, 10 };

static auto entryRatesRoll    = OSD_FLOAT_t { &data.rates.rates[FlightController::FD_ROLL], 1, 255, 1, 10 };
static auto entryRatesPitch   = OSD_FLOAT_t { &data.rates.rates[FlightController::FD_PITCH], 1, 255, 1, 10 };
static auto entryRatesYaw     = OSD_FLOAT_t { &data.rates.rates[FlightController::FD_YAW], 1, 255, 1, 10 };

static auto entryRcExpoRoll   = OSD_FLOAT_t { &data.rates.rcRates[FlightController::FD_ROLL], 1, 100, 1, 10 };
static auto entryRcExpoPitch  = OSD_FLOAT_t { &data.rates.rcRates[FlightController::FD_PITCH], 1, 100, 1, 10 };
static auto entryRcExpoYaw    = OSD_FLOAT_t { &data.rates.rcRates[FlightController::FD_YAW], 1, 100, 1, 10 };

static auto entryThrottleMid  = OSD_UINT8_t { &data.rates.throttleMidpoint, 1, 100, 1 };
static auto entryThrottleExpo = OSD_UINT8_t { &data.rates.throttleExpo, 1, 100, 1 };
static auto entryThrottleLimitType = OSD_TABLE_t { &data.rates.throttleLimitType, 2, &lookupTableThrottleLimitType[0] };
static auto entryThrottleLimitPercent = OSD_UINT8_t { &data.rates.throttleLimitPercent, 25, 100, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 16> menuRatesEntries
{{
    { "-- RATE --",  OME_Label, nullptr, &rateProfileIndexString[0] },

    { "ROLL RATE",   OME_FLOAT,  nullptr, &entryRcRatesRoll },
    { "PITCH RATE",  OME_FLOAT,  nullptr, &entryRcRatesPitch },
    { "YAW RATE",    OME_FLOAT,  nullptr, &entryRcRatesYaw },

    { "ROLL SUPER",  OME_FLOAT,  nullptr, &entryRatesRoll },
    { "PITCH SUPER", OME_FLOAT,  nullptr, &entryRatesPitch },
    { "YAW SUPER",   OME_FLOAT,  nullptr, &entryRatesYaw },

    { "ROLL EXPO",   OME_FLOAT,  nullptr, &entryRcExpoRoll },
    { "PITCH EXPO",  OME_FLOAT,  nullptr, &entryRcExpoPitch },
    { "YAW EXPO",    OME_FLOAT,  nullptr, &entryRcExpoYaw },

    { "THR MID",     OME_UINT8,  nullptr, &entryThrottleMid },
    { "THR EXPO",    OME_UINT8,  nullptr, &entryThrottleExpo },

    { "THR LIM TYPE",OME_TABLE,  nullptr, &entryThrottleLimitType },
    { "THR LIM %",   OME_UINT8,  nullptr, &entryThrottleLimitPercent },

    { "BACK", OME_Back, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

static const void* rateProfileIndexOnChange(CMSX& cmsx, DisplayPortBase& displayPort, const CMSX::menu_t* menu)
{
    (void)cmsx;
    (void)displayPort;
    (void)menu;
    return nullptr;
}

CMSX::menu_t CMSX::menuRates = {
    .onEnter = menuRatesOnEnter,
    .onExit = menuRatesOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuRatesEntries[0]
};

static std::array<const char * const, 4> rateProfileNames { "1", "2", "3", "4" };

//
// Profile Menu
//

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryPID_Profile = OSD_TABLE_t { nullptr, 3, &pidProfileNames[0] };
static auto entryRateProfile = OSD_TABLE_t { nullptr, 3, &rateProfileNames[0] };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 8> menuProfileEntries
{{
    {"-- PROFILE --",  OME_Label, nullptr, nullptr},

    {"PID PROF",    OME_TABLE, &pidProfileIndexOnChange, &entryPID_Profile},
    {"PID",         OME_Submenu, &CMSX::menuChange, &CMSX::menuPID},

    {"RATE PROF",   OME_TABLE, &rateProfileIndexOnChange, &entryRateProfile},
    {"RATES",       OME_Submenu, &CMSX::menuChange, &CMSX::menuRates},

    {"FILTERS",     OME_Submenu, &CMSX::menuChange, &CMSX::menuFilters},

    {"BACK",        OME_Back, nullptr, nullptr},
    {nullptr,       OME_END, nullptr, nullptr}
}};

static const void* menuProfileOnEnter(CMSX& cmsx, DisplayPortBase& displayPort, const CMSX::OSD_Entry* entry)
{
    (void)entry;
    CMSX::menuChange(cmsx, displayPort, &CMSX::menuSetupPopup);
    return nullptr;
}

static const void* menuProfileOnExit(CMSX& cmsx, DisplayPortBase& displayPort, const CMSX::OSD_Entry* entry)
{
    (void)entry;
    CMSX::menuChange(cmsx, displayPort, &CMSX::menuSetupPopup);
    return nullptr;
}

CMSX::menu_t CMSX::menuProfile = {
    .onEnter = menuProfileOnEnter,
    .onExit = menuProfileOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuProfileEntries[0]
};
