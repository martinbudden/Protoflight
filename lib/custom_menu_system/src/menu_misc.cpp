#include "cms.h"
#include "cms_types.h"
#include "cockpit.h"
#include "flight_controller.h"
#include <motor_mixer_base.h>
#include <receiver_base.h>


static std::array<uint16_t, 8> rcData;
static uint16_t motorIdle;
static uint8_t fpvCam_angle_degrees;
static uint8_t crashFlipRate;

#if false
static const void* menuRcConfirmBack(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    (void)cmsx;
    if (self && ((self->flags & OME_TYPE_MASK) == OME_BACK)) {
        return nullptr;
    }
    return CMSX::MENU_BACK;
}
#endif

// cppcheck-suppress constParameterCallback
static const void* menuRcOnDisplayUpdate(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    (void)cmsx;
    const ReceiverBase& receiver = ctx.receiver;
    size_t ii = 0;
    for (auto& rc : rcData) {
        rc = receiver.get_channel_pwm(ii);
        ++ii;
    }
    return nullptr;
}

// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryRcRoll     = osd_uint16_t { &rcData[0], 1, 2500, 0 };
static auto entryRcPitch    = osd_uint16_t { &rcData[1], 1, 2500, 0 };
static auto entryRcThrottle = osd_uint16_t { &rcData[2], 1, 2500, 0 };
static auto entryRcYaw      = osd_uint16_t { &rcData[3], 1, 2500, 0 };
static auto entryRcAux1     = osd_uint16_t { &rcData[4], 1, 2500, 0 };
static auto entryRcAux2     = osd_uint16_t { &rcData[5], 1, 2500, 0 };
static auto entryRcAux3     = osd_uint16_t { &rcData[6], 1, 2500, 0 };
static auto entryRcAux4     = osd_uint16_t { &rcData[7], 1, 2500, 0 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 9> menuRcEntries
{{
    { "-- RC PREV --", OME_LABEL, nullptr, nullptr},

    { "ROLL",  OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcRoll },
    { "PITCH", OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcPitch },
    { "THR",   OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcThrottle },
    { "YAW",   OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcYaw },
    { "AUX1",  OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcAux1 },
    { "AUX2",  OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcAux2 },
#if false
    { "AUX3",  OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcAux3 },
    { "AUX4",  OME_UINT16 | OME_DYNAMIC, nullptr, &entryRcAux4 },
#endif
    { "BACK",  OME_BACK, nullptr, nullptr},
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menu_rc_preview {
    .on_enter = CMSX::inhibit_save_menu,
    .on_exit = nullptr,
    .on_display_update = menuRcOnDisplayUpdate,
    .entries = &menuRcEntries[0]
};

// cppcheck-suppress constParameterCallback
static const void* menu_miscOnEnter(CMSX& cmsx, cms_context_t& ctx)
{
    (void)cmsx;
    const MotorMixerBase& motor_mixer = ctx.motor_mixer;
    motorIdle = motor_mixer.get_motor_config().motor_idle;
    fpvCam_angle_degrees = 0;
    crashFlipRate = 0;

    return nullptr;
}

static const void* menu_miscOnExit(CMSX& cmsx, cms_context_t& ctx, [[maybe_unused]] const CMSX::OSD_Entry* self)
{
    (void)cmsx;
    MotorMixerBase& motor_mixer = ctx.motor_mixer;
    motor_config_t motorConfig =  motor_mixer.get_motor_config();
    motorConfig.motor_idle = motorIdle;
    motor_mixer.set_motor_config(motorConfig);

    return nullptr;
}


// NOLINTBEGIN(fuchsia-statically-constructed-objects)
static auto entryMotorIdle      = osd_uint16_t { &motorIdle, 1, 2000, 10 };
static auto entryFpvCamAngle    = osd_uint8_t  { &fpvCam_angle_degrees, 1, 90, 1 };
static auto entryCrashFlipRate  = osd_uint8_t  { &crashFlipRate, 1, 100, 1 };
// NOLINTEND(fuchsia-statically-constructed-objects)

static const std::array<CMSX::OSD_Entry, 8> menu_miscEntries
{{
    { "-- MISC --", OME_LABEL, nullptr, nullptr },

    { "IDLE OFFSET",    OME_UINT16 | OME_REBOOT_REQUIRED,   nullptr, &entryMotorIdle },
    { "FPV CAM ANGLE",  OME_UINT8,                          nullptr, &entryFpvCamAngle },
    { "CRASHFLIP RATE", OME_UINT8  | OME_REBOOT_REQUIRED,   nullptr, &entryCrashFlipRate },
    { "RC PREV",        OME_SUBMENU,                        CMSX::menu_change, &CMSX::menu_rc_preview},
#if defined(USE_GPS_LAP_TIMER)
    { "GPS LAP TIMER",  OME_SUBMENU, CMSX::menu_change, &menuGpsLapTimer },
#endif // USE_GPS_LAP_TIMER

    { "BACK", OME_BACK, nullptr, nullptr},
    { nullptr, OME_END, nullptr, nullptr}
}};

CMSX::menu_t CMSX::menu_misc {
    .on_enter = menu_miscOnEnter,
    .on_exit = menu_miscOnExit,
    .on_display_update = nullptr,
    .entries = &menu_miscEntries[0]
};
