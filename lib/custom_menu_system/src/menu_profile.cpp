#include "cms.h"
#include "cms_types.h"
#include "cockpit.h"
#include "flight_controller.h"
#include "imu_filters.h"
#include "lookup_tables.h"
#include "rates.h"

/*
Storage

Only one menu can be active at one time, so we can save RAM by using a union.
*/
union data_u { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    rates_t rates;

    imu_filters_config_t imu_filtersConfig;

    simplified_pid_settings_t pidSettings;
    std::array<pid_constants_uint16_t, 3> pids;
    flight_controller_filters_config_t pid_filtersConfig;
};

static data_u data {};

static uint8_t rate_profile_index = 0;
static uint8_t pid_profile_index = 0;
static uint8_t pid_tuning_mode;


//
// Rates
//

static std::array<uint8_t, 3> rate_profile_indexString = { 'R', '1', '\0' };

// cppcheck-suppress constParameterCallback
static const void* menu_ratesOnEnter(CMSX& cmsx, cms_context_t& ctx)
{
    (void)cmsx;
    rate_profile_indexString[1] = '1' + rate_profile_index;
    data.rates = ctx.cockpit.get_rates();
    return nullptr;
}

static const void* menu_ratesOnExit(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::osd_entry_t* self)
{
    (void)cmsx;
    ctx.cockpit.set_rates(data.rates);
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)

static auto entryRcRatesRoll  = osd_uint8_t { &data.rates.rc_rates[FlightController::FD_ROLL], 1, 255, 1 };
static auto entryRcRatesPitch = osd_uint8_t { &data.rates.rc_rates[FlightController::FD_PITCH], 1, 255, 1 };
static auto entryRcRatesYaw   = osd_uint8_t { &data.rates.rc_rates[FlightController::FD_YAW], 1, 255, 1 };

static auto entryRatesRoll    = osd_uint8_t { &data.rates.rates[FlightController::FD_ROLL], 1, 255, 1 };
static auto entryRatesPitch   = osd_uint8_t { &data.rates.rates[FlightController::FD_PITCH], 1, 255, 1 };
static auto entryRatesYaw     = osd_uint8_t { &data.rates.rates[FlightController::FD_YAW], 1, 255, 1 };

static auto entryRcExpoRoll   = osd_uint8_t { &data.rates.rc_expos[FlightController::FD_ROLL], 1, 100, 1 };
static auto entryRcExpoPitch  = osd_uint8_t { &data.rates.rc_expos[FlightController::FD_PITCH], 1, 100, 1 };
static auto entryRcExpoYaw    = osd_uint8_t { &data.rates.rc_expos[FlightController::FD_YAW], 1, 100, 1 };

static auto entryThrottleMid  = osd_uint8_t { &data.rates.throttle_midpoint, 1, 100, 1 };
static auto entryThrottleExpo = osd_uint8_t { &data.rates.throttle_expo, 1, 100, 1 };
static_assert(static_cast<int>(LOOKUP_TABLES::THROTTLE_LIMIT_NAMES_COUNT) == static_cast<int>(rates_t::THROTTLE_LIMIT_TYPE_COUNT));
static auto entryThrottleLimitType = osd_table_t { &data.rates.throttle_limit_type, LOOKUP_TABLES::THROTTLE_LIMIT_NAMES_COUNT - 1, &LOOKUP_TABLES::throttle_limit_typeNames[0] };
static auto entryThrottleLimitPercent = osd_uint8_t { &data.rates.throttle_limit_percent, 25, 100, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::osd_entry_t, 10> menu_ratesEntries
{{
    { "-- RATE --",  OME_LABEL, nullptr, &rate_profile_indexString[0] },

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

CMSX::menu_t CMSX::menu_rates = {
    .on_enter = menu_ratesOnEnter,
    .on_exit = menu_ratesOnExit,
    .on_display_update = nullptr,
    .entries = &menu_ratesEntries[0]
};

static const void* rate_profile_indexOnChange(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)cmsx;
    (void)ctx;
    return nullptr;
}

static std::array<const char * const, 4> rateProfileNames { "R1", "R2", "R3", "R4" };

//
// IMU _filters
//

static const void* menu_imu_filters_on_enter(CMSX& cmsx, cms_context_t& ctx)
{
    (void)cmsx;
    data.imu_filtersConfig = ctx.imu_filters.get_config(); // NOLINT(cppcoreguidelines-pro-type-union-access)
    return nullptr;
}

static const void* menu_imu_filters_on_exit(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::osd_entry_t* self)
{
    (void)cmsx;
    ctx.imu_filters.set_config(data.imu_filtersConfig); // NOLINT(cppcoreguidelines-pro-type-union-access)
    return nullptr;
}

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,fuchsia-statically-constructed-objects)
static auto entryGyroLPF1 = osd_uint16_t { &data.imu_filtersConfig.gyro_lpf1_hz, 0, ImuFilters::GYRO_LPF_MAX_HZ, 1 };
static auto entryGyroLPF2 = osd_uint16_t { &data.imu_filtersConfig.gyro_lpf2_hz, 0, ImuFilters::GYRO_LPF_MAX_HZ, 1 };
static auto entryGyroNF1  = osd_uint16_t { &data.imu_filtersConfig.gyro_notch1_hz, 0, 500, 1 };
static auto entryGyroNF1C = osd_uint16_t { &data.imu_filtersConfig.gyro_notch1_cutoff, 0, 500, 1 };
static auto entryGyroNF2  = osd_uint16_t { &data.imu_filtersConfig.gyro_notch2_hz, 0, 500, 1 };
static auto entryGyroNF2C = osd_uint16_t { &data.imu_filtersConfig.gyro_notch2_cutoff, 0, 500, 1 };
// NOLINTEND(cppcoreguidelines-pro-type-union-access,fuchsia-statically-constructed-objects)

static const std::array<CMSX::osd_entry_t, 9> menu_imu_filters_entries
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

CMSX::menu_t CMSX::menu_imu_filters = {
    .on_enter = menu_imu_filters_on_enter,
    .on_exit = menu_imu_filters_on_exit,
    .on_display_update = nullptr,
    .entries = &menu_imu_filters_entries[0]
};

//
// PID _filters
//

// cppcheck-suppress constParameterCallback
static const void* menu_pid_filters_on_enter(CMSX& cmsx, cms_context_t& ctx)
{
    (void)cmsx;
    const FlightController& flight_controller = ctx.flight_controller;
    data.pid_filtersConfig = flight_controller.get_filters_config(); // NOLINT(cppcoreguidelines-pro-type-union-access)
    return nullptr;
}

static const void* menu_pid_filters_on_exit(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::osd_entry_t* self)
{
    (void)cmsx;
    FlightController& flight_controller = ctx.flight_controller;
    flight_controller.set_filters_config(data.pid_filtersConfig); // NOLINT(cppcoreguidelines-pro-type-union-access)
    return nullptr;
}

static const std::array<CMSX::osd_entry_t, 3> menu_pid_filters_entries
{{
    { "-- PID FILTERS --", OME_LABEL, nullptr, nullptr },


    {"BACK",        OME_BACK, nullptr, nullptr},
    {nullptr,       OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menu_pid_filters = {
    .on_enter = menu_pid_filters_on_enter,
    .on_exit = menu_pid_filters_on_exit,
    .on_display_update = nullptr,
    .entries = &menu_pid_filters_entries[0]
};

//
// Simplified PID tuning
//

// cppcheck-suppress constParameterCallback
static const void* menuSimplifiedTuningOnEnter(CMSX& cmsx, cms_context_t& ctx)
{
    (void)cmsx;
    const FlightController& flight_controller = ctx.flight_controller;
    pid_tuning_mode = flight_controller.get_pid_tuning_mode();
    data.pidSettings = flight_controller.get_simplified_pid_Settings(); // NOLINT(cppcoreguidelines-pro-type-union-access)

    return nullptr;
}

static const void* menuSimplifiedTuningOnExit(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::osd_entry_t* self)
{
    (void)cmsx;
    FlightController& flight_controller = ctx.flight_controller;
    flight_controller.set_pid_tuning_mode(static_cast<FlightController::pid_tuning_mode_e>(pid_tuning_mode));
    flight_controller.set_simplified_pid_Settings(data.pidSettings); // NOLINT(cppcoreguidelines-pro-type-union-access)

    return nullptr;
}



// NOLINTBEGIN(fuchsia-statically-constructed-objects,cppcoreguidelines-pro-type-union-access)
static auto entryTable_pid_tuningMode  = osd_table_t  { &pid_tuning_mode,  LOOKUP_TABLES::PID_TUNING_MODES_COUNT - 1, &LOOKUP_TABLES::_pid_tuningModes[0] };

static constexpr uint16_t PID_GAIN_MAX = FlightController::PID_GAIN_MAX;
static auto entryD_gains        = osd_uint16_t { &data.pidSettings.d_gain, 0, PID_GAIN_MAX, 1 };
static auto entryPI_gains       = osd_uint16_t { &data.pidSettings.pi_gain, 0, PID_GAIN_MAX, 1 };
static auto entryK_gains        = osd_uint16_t { &data.pidSettings.k_gain, 0, PID_GAIN_MAX, 1 };
#if defined(USE_DMAX)
static auto entryDMax           = osd_uint16_t { &data.pidSettings.dmax_gain, 0, PID_GAIN_MAX, 1 };
#endif
static auto entryI_gains        = osd_uint16_t { &data.pidSettings.i_gain, 0, PID_GAIN_MAX, 1 };
static auto entryRollPitchRatio = osd_uint16_t { &data.pidSettings.roll_pitch_ratio, 0, PID_GAIN_MAX, 1 };
static auto entryPitchPI_gains  = osd_uint16_t { &data.pidSettings.pitch_pi_gain, 0, PID_GAIN_MAX, 1 };
static auto entryMasterMultiplier = osd_uint16_fixed_t { &data.pidSettings.multiplier, 10, PID_GAIN_MAX, 5, 10 };
// NOLINTEND(fuchsia-statically-constructed-objects,cppcoreguidelines-pro-type-union-access)

#if defined(USE_DMAX)
static const std::array<CMSX::osd_entry_t, 14> menuSimplifiedTuningEntries
#else
static const std::array<CMSX::osd_entry_t, 13> menuSimplifiedTuningEntries
#endif
{{
    { "-- SIMPLIFIED TUNE --", OME_LABEL, nullptr, nullptr},
    { "PID TUNING",        OME_TABLE,  nullptr, &entryTable_pid_tuningMode },

    { "-- BASIC --",       OME_LABEL,  nullptr, nullptr},
    { "D GAINS",           OME_UINT16, nullptr, &entryD_gains },
    { "P&I GAINS",         OME_UINT16, nullptr, &entryPI_gains },
    { "FF GAINS",          OME_UINT16, nullptr, &entryK_gains },

    { "-- EXPERT --",      OME_LABEL,  nullptr, nullptr},
#if defined(USE_DMAX)
    { "D MAX",             OME_UINT16, nullptr, &entryDMax },
#endif
    { "I GAINS",           OME_UINT16, nullptr, &entryI_gains },

    { "PITCH:ROLL D",      OME_UINT16, nullptr, &entryRollPitchRatio },
    { "PITCH:ROLL P,I&FF", OME_UINT16, nullptr, &entryPitchPI_gains },
    { "MASTER MULT",       OME_UINT16_FIXED, nullptr, &entryMasterMultiplier },

    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menu_simplified_pid_Tuning = {
    .on_enter = menuSimplifiedTuningOnEnter,
    .on_exit = menuSimplifiedTuningOnExit,
    .on_display_update = nullptr,
    .entries = &menuSimplifiedTuningEntries[0]
};

//
// PID Tuning
//

static std::array<uint8_t, 3> pid_profile_indexString = { 'P', '1', '\0' };

// cppcheck-suppress constParameterCallback
static const void* menu_pid_tuningOnEnter(CMSX& cmsx, cms_context_t& ctx)
{
    (void)cmsx;
    pid_profile_indexString[1] = '1' + cmsx.get_current_pid_profile_index(ctx);
    const FlightController& flight_controller = ctx.flight_controller;
    data.pids[FlightController::ROLL_RATE_DPS] = flight_controller.get_pid_constants(FlightController::ROLL_RATE_DPS);
    data.pids[FlightController::PITCH_RATE_DPS] = flight_controller.get_pid_constants(FlightController::PITCH_RATE_DPS);
    data.pids[FlightController::YAW_RATE_DPS] = flight_controller.get_pid_constants(FlightController::YAW_RATE_DPS);
    return nullptr;
}

static const void* menu_pid_tuningOnExit(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::osd_entry_t* self)
{
    (void)cmsx;
    FlightController& flight_controller = ctx.flight_controller;
    flight_controller.set_pid_constants(FlightController::ROLL_RATE_DPS, data.pids[FlightController::ROLL_RATE_DPS]);
    flight_controller.set_pid_constants(FlightController::PITCH_RATE_DPS, data.pids[FlightController::PITCH_RATE_DPS]);
    flight_controller.set_pid_constants(FlightController::YAW_RATE_DPS, data.pids[FlightController::YAW_RATE_DPS]);
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

static const std::array<CMSX::osd_entry_t, 18> menu_pid_tuningEntries
{{
    { "-- PID --", OME_LABEL, nullptr, &pid_profile_indexString[0] },

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

CMSX::menu_t CMSX::menu_pid_tuning = {
    .on_enter = menu_pid_tuningOnEnter,
    .on_exit = menu_pid_tuningOnExit,
    .on_display_update = nullptr,
    .entries = &menu_pid_tuningEntries[0]
};

static const void* pid_profile_indexOnChange(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)cmsx;
    (void)ctx;
    return nullptr;
}

static std::array<const char * const, 4> pid_profileNames { "P1", "P2", "P3", "P4" };

//
// Profile Menu
//

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryRateProfile = osd_table_t { &rate_profile_index, 4-1, &rateProfileNames[0] };
static auto entryPID_Profile = osd_table_t { &pid_profile_index, 4-1, &pid_profileNames[0] };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::osd_entry_t, 10> menu_profileEntries
{{
    {"-- PROFILES --",  OME_LABEL, nullptr, nullptr},

    {"RATE PROFILE",    OME_TABLE, &rate_profile_indexOnChange, &entryRateProfile},
    {"RATES",           OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_rates},

    {"PID PROFILE",     OME_TABLE, &pid_profile_indexOnChange, &entryPID_Profile},
    {"PID",             OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_pid_tuning},
    {"SIMPLIFIED PIDS", OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_simplified_pid_Tuning },
    {"PID FILTERS",     OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_pid_filters},

    {"IMU FILTERS",     OME_SUBMENU, &CMSX::menu_change, &CMSX::menu_imu_filters},

    {"BACK",            OME_BACK, nullptr, nullptr},
    {nullptr,           OME_END, nullptr, nullptr}
}};

// cppcheck-suppress constParameterCallback
static const void* menu_profileOnEnter(CMSX& cmsx, cms_context_t& ctx)
{
    pid_profile_index = cmsx.get_current_pid_profile_index(ctx);
    rate_profile_index = cmsx.get_current_rate_profile_index(ctx);

    //CMSX::menu_change(cmsx, display_port, &CMSX::menu_setup_popup);
    return nullptr;
}

static const void* menu_profileOnExit(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::osd_entry_t* self)
{
    cmsx.set_current_pid_profile_index(ctx, pid_profile_index);
    cmsx.set_current_rate_profile_index(ctx, rate_profile_index);

    //CMSX::menu_change(cmsx, display_port, &CMSX::menu_setup_popup);
    return nullptr;
}

CMSX::menu_t CMSX::menu_profile = {
    .on_enter = menu_profileOnEnter,
    .on_exit = menu_profileOnExit,
    .on_display_update = nullptr,
    .entries = &menu_profileEntries[0]
};
