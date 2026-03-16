#include "cms_types.h"
#include "cmsx.h"
//#include "version.h"
#include <imu_base.h>

std::array<char, CMSX::CALIBRATION_STATUS_MAX_LENGTH> CMSX::GyroCalibrationStatus {};
std::array<char, CMSX::CALIBRATION_STATUS_MAX_LENGTH> CMSX::AccCalibrationStatus {};
#if defined(USE_BAROMETER)
std::array<char, CMSX::CALIBRATION_STATUS_MAX_LENGTH> CMSX::BaroCalibrationStatus {};
#endif

const void* CMSX::menu_calibrate_gyro(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    CMSX::menu_change(cmsx, ctx, nullptr);//&menu_calibrate_gyro);
    return nullptr;
}

static const void* calibrate_acc(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    (void)cmsx;
    (void)ctx;
#if defined(FRAMEWORK_TEST)
    (void)cmsx;
#else
    enum { CALIBRATION_COUNT = 5000 };
    ctx.imu.calibrate(ImuBase::CALIBRATE_ACC_AND_GYRO, CALIBRATION_COUNT);
#endif

    return CMSX::MENU_BACK;
}

static const std::array<CMSX::OSD_Entry, 7> menu_calibrate_accEntries
{{
    { "-- CALIBRATE ACC --",      OME_LABEL, nullptr, nullptr },
    { "PLACE ON A LEVEL SURFACE",   OME_LABEL, nullptr, nullptr},
    { "MAKE SURE CRAFT IS STILL",   OME_LABEL, nullptr, nullptr},
    { " ",                          OME_LABEL, nullptr, nullptr},
    { "START CALIBRATION",          OME_FUNCTION_CALL, calibrate_acc, nullptr },
    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

const void* CMSX::menu_calibrate_acc(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    CMSX::menu_change(cmsx, ctx, nullptr);//&menu_calibrate_acc);
    return nullptr;
}

const void* CMSX::menu_calibrate_baro(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::menu_t* menu)
{
    CMSX::menu_change(cmsx, ctx, nullptr);//&menu_calibrate_baro);
    return nullptr;
}

static const std::array<CMSX::OSD_Entry, 7> menu_calibrateEntries
{{
    { "-- CALIBRATE --", OME_LABEL, nullptr, nullptr },
    { "GYRO",   OME_FUNCTION_CALL | OME_DYNAMIC, CMSX::menu_calibrate_gyro, &CMSX::GyroCalibrationStatus[0] },
    { "ACC",    OME_FUNCTION_CALL | OME_DYNAMIC, CMSX::menu_calibrate_acc, &CMSX::AccCalibrationStatus[0] },
#if defined(USE_BAROMETER)
    { "BARO",   OME_FUNCTION_CALL | OME_DYNAMIC, CMSX::menu_calibrate_baro, &CMSX::BaroCalibrationStatus[0] },
#endif
    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

static const void* menu_calibrateOnDisplayUpdate(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::OSD_Entry* selected)
{
    (void)cmsx;
    (void)ctx;
    return nullptr;
}

CMSX::menu_t CMSX::menu_calibrate {
    .on_enter = nullptr,
    .on_exit = nullptr,
    .on_display_update = menu_calibrateOnDisplayUpdate,
    .entries = &menu_calibrateEntries[0]
};

// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define FC_FIRMWARE_NAME            "Betaflight"
#define FC_FIRMWARE_IDENTIFIER      "BTFL"
#define FC_VERSION_STRING           "4.5.0"
// NOLINTEND(cppcoreguidelines-macro-usage)

static const std::array<CMSX::OSD_Entry, 7> menu_firmwareEntries
{{
    { "-- INFO --", OME_LABEL, nullptr, nullptr },

    { "FW ID",   OME_STRING, nullptr, FC_FIRMWARE_IDENTIFIER },
    { "FW VER",  OME_STRING, nullptr, FC_VERSION_STRING },
    //{ "GITREV", OME_STRING, nullptr, __REVISION__ },
    //{ "TARGET", OME_STRING, nullptr, __TARGET__ },
#if defined(USE_BOARD_INFO)
    { "MFR",    OME_STRING, nullptr, manufacturerId },
    { "BOARD",  OME_STRING, nullptr, boardName },
#endif
    { "-- SETUP --", OME_LABEL, nullptr, nullptr },
    { "CALIBRATE",     OME_SUBMENU, CMSX::menu_change, &CMSX::menu_calibrate},
    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menu_firmware {
#if defined(USE_BOARD_INFO)
    .on_enter = menu_firmwareOnEnter,
#else
    .on_enter = nullptr,
#endif
    .on_exit = nullptr,
    .on_display_update = nullptr,
    .entries = &menu_firmwareEntries[0]
};
