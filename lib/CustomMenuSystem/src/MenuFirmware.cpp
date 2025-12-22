#include "CMSX.h"
#include "CMS_Types.h"
#include "version.h"
#include <IMU_Base.h>

std::array<char, CMSX::CALIBRATION_STATUS_MAX_LENGTH> CMSX::GyroCalibrationStatus {};
std::array<char, CMSX::CALIBRATION_STATUS_MAX_LENGTH> CMSX::AccCalibrationStatus {};


const void* CMSX::menuCalibrateGyro(CMSX& cmsx, DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::menu_t* menu)
{
    CMSX::menuChange(cmsx, displayPort, nullptr);//&menuCalibrateGyro);
    return nullptr;
}

static const void* calibrateAcc(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::menu_t* menu)
{
#if defined(FRAMEWORK_TEST)
    (void)cmsx;
#else
    enum { CALIBRATION_COUNT = 5000 };
    cmsx.getIMU().calibrate(IMU_Base::CALIBRATE_ACC_AND_GYRO, CALIBRATION_COUNT);
#endif

    return CMSX::MENU_BACK;
}

static const std::array<CMSX::OSD_Entry, 7> menuCalibrateAccEntries
{{
    { "-- CALIBRATE ACC --",      OME_LABEL, nullptr, nullptr },
    { "PLACE ON A LEVEL SURFACE",   OME_LABEL, nullptr, nullptr},
    { "MAKE SURE CRAFT IS STILL",   OME_LABEL, nullptr, nullptr},
    { " ",                          OME_LABEL, nullptr, nullptr},
    { "START CALIBRATION",          OME_FUNCTION_CALL, calibrateAcc, nullptr },
    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

const void* CMSX::menuCalibrateAcc(CMSX& cmsx, DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::menu_t* menu)
{
    CMSX::menuChange(cmsx, displayPort, nullptr);//&menuCalibrateAcc);
    return nullptr;
}

static const std::array<CMSX::OSD_Entry, 7> menuCalibrateEntries
{{
    { "-- CALIBRATE --", OME_LABEL, nullptr, nullptr },
    { "GYRO",   OME_FUNCTION_CALL | OME_DYNAMIC, CMSX::menuCalibrateGyro, &CMSX::GyroCalibrationStatus[0] },
    { "ACC",    OME_FUNCTION_CALL | OME_DYNAMIC, CMSX::menuCalibrateAcc, &CMSX::AccCalibrationStatus[0] },
#if defined(USE_BAROMETER)
    { "BARO",   OME_FUNCTION_CALL | OME_DYNAMIC, menuCalibrateBaro, &CMSX::baroCalibrationStatus },
#endif
    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

static const void* menuCalibrateOnDisplayUpdate(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* selected)
{
    (void)cmsx;
    return nullptr;
}

CMSX::menu_t CMSX::menuCalibrate {
    .onEnter = nullptr,
    .onExit = nullptr,
    .onDisplayUpdate = menuCalibrateOnDisplayUpdate,
    .entries = &menuCalibrateEntries[0]
};

static const std::array<CMSX::OSD_Entry, 7> menuFirmwareEntries
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
    { "CALIBRATE",     OME_SUBMENU, CMSX::menuChange, &CMSX::menuCalibrate},
    { "BACK", OME_BACK, nullptr, nullptr },
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuFirmware {
#if defined(USE_BOARD_INFO)
    .onEnter = menuFirmwareOnEnter,
#else
    .onEnter = nullptr,
#endif
    .onExit = nullptr,
    .onDisplayUpdate = nullptr,
    .entries = &menuFirmwareEntries[0]
};
