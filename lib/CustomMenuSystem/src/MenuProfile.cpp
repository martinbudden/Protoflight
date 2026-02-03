#include "CMS.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "IMU_Filters.h"
#include "LookupTables.h"
#include "Rates.h"

/*
Storage

Only one menu can be active at one time, so we can save RAM by using a union.
*/
union data_u { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    rates_t rates;

    IMU_Filters::config_t imuFiltersConfig;

    FlightController::simplified_pid_settings_t pidSettings;
    std::array<FlightController::PIDF_uint16_t, 3> pids;
    FlightController::filters_config_t pidFiltersConfig;
};

static data_u data {};

static uint8_t rateProfileIndex = 0;
static uint8_t pidProfileIndex = 0;
static uint8_t pidTuningMode;


//
// Rates
//

static std::array<uint8_t, 3> rateProfileIndexString = { 'R', '1', '\0' };

static const void* menuRatesOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    rateProfileIndexString[1] = '1' + rateProfileIndex;
    data.rates = cmsx.getCMS().getCockpit().getRates();
    return nullptr;
}

static const void* menuRatesOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    cmsx.getCMS().getCockpit().setRates(data.rates, cmsx.getCockpit().getFlightControllerMutable());
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)

static auto entryRcRatesRoll  = osd_uint8_t { &data.rates.rcRates[FlightController::FD_ROLL], 1, 255, 1 };
static auto entryRcRatesPitch = osd_uint8_t { &data.rates.rcRates[FlightController::FD_PITCH], 1, 255, 1 };
static auto entryRcRatesYaw   = osd_uint8_t { &data.rates.rcRates[FlightController::FD_YAW], 1, 255, 1 };

static auto entryRatesRoll    = osd_uint8_t { &data.rates.rates[FlightController::FD_ROLL], 1, 255, 1 };
static auto entryRatesPitch   = osd_uint8_t { &data.rates.rates[FlightController::FD_PITCH], 1, 255, 1 };
static auto entryRatesYaw     = osd_uint8_t { &data.rates.rates[FlightController::FD_YAW], 1, 255, 1 };

static auto entryRcExpoRoll   = osd_uint8_t { &data.rates.rcExpos[FlightController::FD_ROLL], 1, 100, 1 };
static auto entryRcExpoPitch  = osd_uint8_t { &data.rates.rcExpos[FlightController::FD_PITCH], 1, 100, 1 };
static auto entryRcExpoYaw    = osd_uint8_t { &data.rates.rcExpos[FlightController::FD_YAW], 1, 100, 1 };

static auto entryThrottleMid  = osd_uint8_t { &data.rates.throttleMidpoint, 1, 100, 1 };
static auto entryThrottleExpo = osd_uint8_t { &data.rates.throttleExpo, 1, 100, 1 };
static_assert(static_cast<int>(LOOKUP_TABLES::THROTTLE_LIMIT_NAMES_COUNT) == static_cast<int>(rates_t::THROTTLE_LIMIT_TYPE_COUNT));
static auto entryThrottleLimitType = osd_table_t { &data.rates.throttleLimitType, LOOKUP_TABLES::THROTTLE_LIMIT_NAMES_COUNT - 1, &LOOKUP_TABLES::throttleLimitTypeNames[0] };
static auto entryThrottleLimitPercent = osd_uint8_t { &data.rates.throttleLimitPercent, 25, 100, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 10> menuRatesEntries
{{
    { "-- RATE --",  OME_LABEL, nullptr, &rateProfileIndexString[0] },

    { "ROLL RATE",   OME_UINT8,  nullptr, &entryRcRatesRoll },
    { "PITCH RATE",  OME_UINT8,  nullptr, &entryRcRatesPitch },
    { "YAW RATE",    OME_UINT8,  nullptr, &entryRcRatesYaw },
#if false
    { "ROLL SUPER",  OME_UINT8,  nullptr, &entryRatesRoll },
    { "PITCH SUPER", OME_UINT8,  nullptr, &entryRatesPitch },
    { "YAW SUPER",   OME_UINT8,  nullptr, &entryRatesYaw },

    { "ROLL EXPO",   OME_UINT8,  nullptr, &entryRcExpoRoll },
    { "PITCH EXPO",  OME_UINT8,  nullptr, &entryRcExpoPitch },
    { "YAW EXPO",    OME_UINT8,  nullptr, &entryRcExpoYaw },
#endif
    { "THR MID",     OME_UINT8,  nullptr, &entryThrottleMid },
    { "THR EXPO",    OME_UINT8,  nullptr, &entryThrottleExpo },

    { "THR LM TYP",  OME_TABLE,  nullptr, &entryThrottleLimitType },
    { "THR LM %",    OME_UINT8,  nullptr, &entryThrottleLimitPercent },

    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuRates = {
    .onEnter = menuRatesOnEnter,
    .onExit = menuRatesOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuRatesEntries[0]
};

static const void* rateProfileIndexOnChange(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)cmsx;
    return nullptr;
}

static std::array<const char * const, 4> rateProfileNames { "R1", "R2", "R3", "R4" };

//
// IMU Filters
//

static const void* menuIMU_FiltersOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    data.imuFiltersConfig = cmsx.getIMU_Filters().getConfig(); // NOLINT(cppcoreguidelines-pro-type-union-access)
    return nullptr;
}

static const void* menuIMU_FiltersOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    cmsx.getIMU_Filters().setConfig(data.imuFiltersConfig); // NOLINT(cppcoreguidelines-pro-type-union-access)
    return nullptr;
}

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,fuchsia-statically-constructed-objects)
static auto entryGyroLPF1 = osd_uint16_t { &data.imuFiltersConfig.gyro_lpf1_hz, 0, IMU_Filters::GYRO_LPF_MAX_HZ, 1 };
static auto entryGyroLPF2 = osd_uint16_t { &data.imuFiltersConfig.gyro_lpf2_hz, 0, IMU_Filters::GYRO_LPF_MAX_HZ, 1 };
static auto entryGyroNF1  = osd_uint16_t { &data.imuFiltersConfig.gyro_notch1_hz, 0, 500, 1 };
static auto entryGyroNF1C = osd_uint16_t { &data.imuFiltersConfig.gyro_notch1_cutoff, 0, 500, 1 };
static auto entryGyroNF2  = osd_uint16_t { &data.imuFiltersConfig.gyro_notch2_hz, 0, 500, 1 };
static auto entryGyroNF2C = osd_uint16_t { &data.imuFiltersConfig.gyro_notch2_cutoff, 0, 500, 1 };
// NOLINTEND(cppcoreguidelines-pro-type-union-access,fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 9> menuIMU_FiltersEntries
{{
    { "-- FILTERS --", OME_LABEL, nullptr, nullptr },

    { "GYRO LPF1",  OME_UINT16 | OME_SLIDER_GYRO, nullptr, &entryGyroLPF1 },
    { "GYRO LPF2",  OME_UINT16 | OME_SLIDER_GYRO, nullptr, &entryGyroLPF2 },
    { "GYRO NF1",   OME_UINT16, nullptr, &entryGyroNF1 },
    { "GYRO NF1C",  OME_UINT16, nullptr, &entryGyroNF1C },
    { "GYRO NF2",   OME_UINT16, nullptr, &entryGyroNF2 },
    { "GYRO NF2C",  OME_UINT16, nullptr, &entryGyroNF2C },

    {"BACK",        OME_BACK, nullptr, nullptr},
    {nullptr,       OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuIMU_Filters = {
    .onEnter = menuIMU_FiltersOnEnter,
    .onExit = menuIMU_FiltersOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuIMU_FiltersEntries[0]
};

//
// PID Filters
//

static const void* menuPID_FiltersOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    const FlightController& flightController = cmsx.getCockpit().getFlightController();
    data.pidFiltersConfig = flightController.getFiltersConfig(); // NOLINT(cppcoreguidelines-pro-type-union-access)
    return nullptr;
}

static const void* menuPID_FiltersOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    FlightController& flightController = cmsx.getCockpit().getFlightControllerMutable();
    flightController.setFiltersConfig(data.pidFiltersConfig); // NOLINT(cppcoreguidelines-pro-type-union-access)
    return nullptr;
}

static const std::array<CMSX::OSD_Entry, 3> menuPID_FiltersEntries
{{
    { "-- PID FILTERS --", OME_LABEL, nullptr, nullptr },


    {"BACK",        OME_BACK, nullptr, nullptr},
    {nullptr,       OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuPID_Filters = {
    .onEnter = menuPID_FiltersOnEnter,
    .onExit = menuPID_FiltersOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuPID_FiltersEntries[0]
};

//
// Simplified PID tuning
//

static const void* menuSimplifiedTuningOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    const FlightController& flightController = cmsx.getCMS().getCockpit().getFlightController();
    pidTuningMode = flightController.getPID_TuningMode();
    data.pidSettings = flightController.getSimplifiedPID_Settings(); // NOLINT(cppcoreguidelines-pro-type-union-access)

    return nullptr;
}

static const void* menuSimplifiedTuningOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    FlightController& flightController = cmsx.getCMS().getCockpit().getFlightControllerMutable();
    flightController.setPID_TuningMode(static_cast<FlightController::pid_tuning_mode_e>(pidTuningMode));
    flightController.setSimplifiedPID_Settings(data.pidSettings); // NOLINT(cppcoreguidelines-pro-type-union-access)

    return nullptr;
}



// NOLINTBEGIN(fuchsia-statically-constructed-objects,cppcoreguidelines-pro-type-union-access)
static auto entryTablePID_TuningMode  = osd_table_t  { &pidTuningMode,  LOOKUP_TABLES::PID_TUNING_MODES_COUNT - 1, &LOOKUP_TABLES::PID_TuningModes[0] };

static constexpr uint16_t PID_GAIN_MAX = FlightController::PID_GAIN_MAX;
static auto entryD_gains        = osd_uint16_t { &data.pidSettings.d_gain, 0, PID_GAIN_MAX, 1 };
static auto entryPI_gains       = osd_uint16_t { &data.pidSettings.pi_gain, 0, PID_GAIN_MAX, 1 };
static auto entryK_gains        = osd_uint16_t { &data.pidSettings.k_gain, 0, PID_GAIN_MAX, 1 };
#if defined(USE_D_MAX)
static auto entryDMax           = osd_uint16_t { &data.pidSettings.d_max_gain, 0, PID_GAIN_MAX, 1 };
#endif
static auto entryI_gains        = osd_uint16_t { &data.pidSettings.i_gain, 0, PID_GAIN_MAX, 1 };
static auto entryRollPitchRatio = osd_uint16_t { &data.pidSettings.roll_pitch_ratio, 0, PID_GAIN_MAX, 1 };
static auto entryPitchPI_gains  = osd_uint16_t { &data.pidSettings.pitch_pi_gain, 0, PID_GAIN_MAX, 1 };
static auto entryMasterMultiplier = osd_uint16_fixed_t { &data.pidSettings.multiplier, 10, PID_GAIN_MAX, 5, 10 };
// NOLINTEND(fuchsia-statically-constructed-objects,cppcoreguidelines-pro-type-union-access)

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
    { "MASTER MULT",       OME_UINT16_FIXED, nullptr, &entryMasterMultiplier },

    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuSimplifiedPID_Tuning = {
    .onEnter = menuSimplifiedTuningOnEnter,
    .onExit = menuSimplifiedTuningOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuSimplifiedTuningEntries[0]
};

//
// PID Tuning
//

static std::array<uint8_t, 3> pidProfileIndexString = { 'P', '1', '\0' };

static const void* menuPID_TuningOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    pidProfileIndexString[1] = '1' + cmsx.getCockpit().getCurrentPidProfileIndex();
    const FlightController& flightController = cmsx.getCockpit().getFlightController();
    data.pids[FlightController::ROLL_RATE_DPS] = flightController.getPID_Constants(FlightController::ROLL_RATE_DPS);
    data.pids[FlightController::PITCH_RATE_DPS] = flightController.getPID_Constants(FlightController::PITCH_RATE_DPS);
    data.pids[FlightController::YAW_RATE_DPS] = flightController.getPID_Constants(FlightController::YAW_RATE_DPS);
    return nullptr;
}

static const void* menuPID_TuningOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    FlightController& flightController = cmsx.getCMS().getCockpit().getFlightControllerMutable();
    flightController.setPID_Constants(FlightController::ROLL_RATE_DPS, data.pids[FlightController::ROLL_RATE_DPS]);
    flightController.setPID_Constants(FlightController::PITCH_RATE_DPS, data.pids[FlightController::PITCH_RATE_DPS]);
    flightController.setPID_Constants(FlightController::YAW_RATE_DPS, data.pids[FlightController::YAW_RATE_DPS]);
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryRollPID_P  = osd_uint16_t { &data.pids[0].kp, 0, PID_GAIN_MAX, 1 };
static auto entryRollPID_I  = osd_uint16_t { &data.pids[0].ki, 0, PID_GAIN_MAX, 1 };
static auto entryRollPID_D  = osd_uint16_t { &data.pids[0].kd, 0, PID_GAIN_MAX, 1 };
static auto entryRollPID_K  = osd_uint16_t { &data.pids[0].kk, 0, PID_GAIN_MAX, 1 };
static auto entryRollPID_S  = osd_uint16_t { &data.pids[0].ks, 0, PID_GAIN_MAX, 1 };

static auto entryPitchPID_P = osd_uint16_t { &data.pids[1].kp, 0, PID_GAIN_MAX, 1 };
static auto entryPitchPID_I = osd_uint16_t { &data.pids[1].ki, 0, PID_GAIN_MAX, 1 };
static auto entryPitchPID_D = osd_uint16_t { &data.pids[1].kd, 0, PID_GAIN_MAX, 1 };
static auto entryPitchPID_K = osd_uint16_t { &data.pids[1].kk, 0, PID_GAIN_MAX, 1 };
static auto entryPitchPID_S = osd_uint16_t { &data.pids[1].ks, 0, PID_GAIN_MAX, 1 };

static auto entryYawPID_P   = osd_uint16_t { &data.pids[2].kp, 0, PID_GAIN_MAX, 1 };
static auto entryYawPID_I   = osd_uint16_t { &data.pids[2].ki, 0, PID_GAIN_MAX, 1 };
static auto entryYawPID_D   = osd_uint16_t { &data.pids[2].kd, 0, PID_GAIN_MAX, 1 };
static auto entryYawPID_K   = osd_uint16_t { &data.pids[2].kk, 0, PID_GAIN_MAX, 1 };
static auto entryYawPID_S   = osd_uint16_t { &data.pids[2].ks, 0, PID_GAIN_MAX, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 18> menuPID_TuningEntries
{{
    { "-- PID --", OME_LABEL, nullptr, &pidProfileIndexString[0] },

    { "ROLL  P", OME_UINT16 | OME_SLIDER_RP, nullptr, &entryRollPID_P },
    { "ROLL  I", OME_UINT16 | OME_SLIDER_RP, nullptr, &entryRollPID_I },
    { "ROLL  D", OME_UINT16 | OME_SLIDER_RP, nullptr, &entryRollPID_D },
    { "ROLL  K", OME_UINT16 | OME_SLIDER_RP, nullptr, &entryRollPID_K },
    { "ROLL  S", OME_UINT16 | OME_SLIDER_RP, nullptr, &entryRollPID_S },

    { "PITCH P", OME_UINT16 | OME_SLIDER_RP, nullptr, &entryPitchPID_P },
    { "PITCH I", OME_UINT16 | OME_SLIDER_RP, nullptr, &entryPitchPID_I },
    { "PITCH D", OME_UINT16 | OME_SLIDER_RP, nullptr, &entryPitchPID_D },
    { "PITCH K", OME_UINT16 | OME_SLIDER_RP, nullptr, &entryPitchPID_K },
    { "PITCH S", OME_UINT16 | OME_SLIDER_RP, nullptr, &entryPitchPID_S },

    { "YAW   P", OME_UINT16 | OME_SLIDER_RPY, nullptr, &entryYawPID_P },
    { "YAW   I", OME_UINT16 | OME_SLIDER_RPY, nullptr, &entryYawPID_I },
    { "YAW   D", OME_UINT16 | OME_SLIDER_RPY, nullptr, &entryYawPID_D },
    { "YAW   K", OME_UINT16 | OME_SLIDER_RPY, nullptr, &entryYawPID_K },
    { "YAW   S", OME_UINT16 | OME_SLIDER_RPY, nullptr, &entryYawPID_S },

    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuPID_Tuning = {
    .onEnter = menuPID_TuningOnEnter,
    .onExit = menuPID_TuningOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuPID_TuningEntries[0]
};

static const void* pidProfileIndexOnChange(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)cmsx;
    return nullptr;
}

static std::array<const char * const, 4> pidProfileNames { "P1", "P2", "P3", "P4" };

//
// Profile Menu
//

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryRateProfile = osd_table_t { &rateProfileIndex, 4-1, &rateProfileNames[0] };
static auto entryPID_Profile = osd_table_t { &pidProfileIndex, 4-1, &pidProfileNames[0] };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 10> menuProfileEntries
{{
    {"-- PROFILES --",  OME_LABEL, nullptr, nullptr},

    {"RATE PROFILE",    OME_TABLE, &rateProfileIndexOnChange, &entryRateProfile},
    {"RATES",           OME_SUBMENU, &CMSX::menuChange, &CMSX::menuRates},

    {"PID PROFILE",     OME_TABLE, &pidProfileIndexOnChange, &entryPID_Profile},
    {"PID",             OME_SUBMENU, &CMSX::menuChange, &CMSX::menuPID_Tuning},
    {"SIMPLIFIED PIDS", OME_SUBMENU, &CMSX::menuChange, &CMSX::menuSimplifiedPID_Tuning },
    {"PID FILTERS",     OME_SUBMENU, &CMSX::menuChange, &CMSX::menuPID_Filters},

    {"IMU FILTERS",     OME_SUBMENU, &CMSX::menuChange, &CMSX::menuIMU_Filters},

    {"BACK",            OME_BACK, nullptr, nullptr},
    {nullptr,           OME_END, nullptr, nullptr}
}};

static const void* menuProfileOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    pidProfileIndex = cmsx.getCockpit().getCurrentPidProfileIndex();
    rateProfileIndex = cmsx.getCockpit().getCurrentRateProfileIndex();

    //CMSX::menuChange(cmsx, displayPort, &CMSX::menuSetupPopup);
    return nullptr;
}

static const void* menuProfileOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    cmsx.getCockpit().setCurrentPidProfileIndex(pidProfileIndex);
    cmsx.getCockpit().setCurrentRateProfileIndex(rateProfileIndex);

    //CMSX::menuChange(cmsx, displayPort, &CMSX::menuSetupPopup);
    return nullptr;
}

CMSX::menu_t CMSX::menuProfile = {
    .onEnter = menuProfileOnEnter,
    .onExit = menuProfileOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuProfileEntries[0]
};
