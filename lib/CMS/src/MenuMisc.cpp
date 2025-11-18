#include "CMS.h"
#include "CMS_Types.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "MotorMixerBase.h"
#include <ReceiverBase.h>


static std::array<uint16_t, 8> rcData;
static uint16_t motorIdle;
static uint8_t fpvCamAngleDegrees;
static uint8_t crashFlipRate;


static const void* menuRcOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    //inhibitSaveMenu();
    (void)cmsx;
    return nullptr;
}

static const void* menuRcConfirmBack(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    (void)cmsx;
    if (self && ((self->flags & OME_TYPE_MASK) == OME_BACK)) {
        return nullptr;
    }
    return CMSX::MENU_BACK;
}

static const void* menuRcOnDisplayUpdate(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    const ReceiverBase& receiver = cmsx.getCMS().getCockpit().getReceiver();
    size_t ii = 0;
    for (auto& rc : rcData) {
        rc = receiver.getChannelRaw(ii);
        ++ii;
    }
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryRcRoll     = OSD_UINT16_t { &rcData[0], 1, 2500, 0 };
static auto entryRcPitch    = OSD_UINT16_t { &rcData[1], 1, 2500, 0 };
static auto entryRcThrottle = OSD_UINT16_t { &rcData[2], 1, 2500, 0 };
static auto entryRcYaw      = OSD_UINT16_t { &rcData[3], 1, 2500, 0 };
static auto entryRcAux1     = OSD_UINT16_t { &rcData[4], 1, 2500, 0 };
static auto entryRcAux2     = OSD_UINT16_t { &rcData[5], 1, 2500, 0 };
static auto entryRcAux3     = OSD_UINT16_t { &rcData[6], 1, 2500, 0 };
static auto entryRcAux4     = OSD_UINT16_t { &rcData[7], 1, 2500, 0 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 11> menuRcEntries
{{
    { "-- RC PREV --", OME_LABEL, nullptr, nullptr},

    { "ROLL",  OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcRoll },
    { "PITCH", OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcPitch },
    { "THR",   OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcThrottle },
    { "YAW",   OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcYaw },
    { "AUX1",  OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcAux1 },
    { "AUX2",  OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcAux2 },
    { "AUX3",  OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcAux3 },
    { "AUX4",  OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcAux4 },

    { "BACK",  OME_BACK, nullptr, nullptr},
    { nullptr, OME_END, nullptr, nullptr}
}};

static CMSX::menu_t menuRcPreview {
    .onEnter = menuRcOnEnter,
    .onExit = menuRcConfirmBack,
    .onDisplayUpdate = menuRcOnDisplayUpdate,
    .entries = &menuRcEntries[0]
};

static const void* menuMiscOnEnter(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort)
{
    const MotorMixerBase& motorMixer = cmsx.getCMS().getCockpit().getFlightController().getMotorMixer();
    motorIdle = motorMixer.getMotorConfig().motorIdle;
    fpvCamAngleDegrees = 0;
    crashFlipRate = 0;

    return nullptr;
}

static const void* menuMiscOnExit(CMSX& cmsx, [[maybe_unused]] DisplayPortBase& displayPort, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    MotorMixerBase& motorMixer = cmsx.getCMS().getCockpit().getFlightController().getMotorMixer();
    MotorMixerBase::motorConfig_t motorConfig =  motorMixer.getMotorConfig();
    motorConfig.motorIdle =  motorIdle;
    motorMixer.setMotorConfig(motorConfig);


    return nullptr;
}


// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryMotorIdle      = OSD_UINT16_t { &motorIdle, 1, 2000, 10 };
static auto entryFpvCamAngle    = OSD_UINT8_t  { &fpvCamAngleDegrees, 1, 90, 1 };
static auto entryCrashFlipRate  = OSD_UINT8_t  { &crashFlipRate, 1, 100, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 8> menuMiscEntries
{{
    { "-- MISC --", OME_LABEL, nullptr, nullptr },

    { "IDLE OFFSET",    OME_UINT16 | OME_REBOOT_REQUIRED,    nullptr, &entryMotorIdle },
    { "FPV CAM ANGLE",  OME_UINT8,                           nullptr, &entryFpvCamAngle },
    { "CRASHFLIP RATE", OME_UINT8  | OME_REBOOT_REQUIRED,    nullptr, &entryCrashFlipRate },
    { "RC PREV",        OME_SUBMENU, CMSX::menuChange, &menuRcPreview},
#if defined(USE_GPS_LAP_TIMER)
    { "GPS LAP TIMER",  OME_SUBMENU, CMSX::menuChange, &menuGpsLapTimer },
#endif // USE_GPS_LAP_TIMER

    { "BACK", OME_BACK, nullptr, nullptr},
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menuMisc {
    .onEnter = menuMiscOnEnter,
    .onExit = menuMiscOnExit,
    .onDisplayUpdate = nullptr,
    .entries = &menuMiscEntries[0]
};
