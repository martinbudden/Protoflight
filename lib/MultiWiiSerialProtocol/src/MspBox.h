#pragma once

#include <array>
#include <bitset>
#include <cstdint>

class StreamBufWriter;

class MspBox {
public:
    MspBox() = default;
public:
    struct box_t {
        const uint8_t id;
        const uint8_t permanent_id; // permanent ID used to identify BOX. This ID is unique for one function, DO NOT REUSE IT
        const char *name;       // GUI-readable box name
    };
    // Each page contains at most 32 boxes
    static constexpr uint8_t MAX_BOXES_PER_PAGE = 32;
    static constexpr uint8_t PERMANENT_ID_NONE = 255;

    // ARM flag
    static constexpr uint8_t BOX_ARM = 0;
    // Flight modes
    static constexpr uint8_t BOX_ANGLE = 1;
    static constexpr uint8_t BOX_HORIZON = 2;
    static constexpr uint8_t BOX_MAG = 3;
    static constexpr uint8_t BOX_ALTITUDE_HOLD = 4;
    static constexpr uint8_t BOX_POSITION_HOLD = 5;
    static constexpr uint8_t BOX_HEADFREE = 6;
    static constexpr uint8_t BOX_CHIRP = 7;
    static constexpr uint8_t BOX_PASSTHRU = 8;
    static constexpr uint8_t BOX_FAILSAFE = 9;
    static constexpr uint8_t BOX_GPS_RESCUE = 10;
    static constexpr uint8_t BOX_ID_FLIGHTMODE_COUNT = 11;

    // When new flight modes are added, the parameter group version for 'modeActivationConditions' in src/main/fc/rc_modes.c has to be incremented to ensure that the RC modes configuration is reset.

    // RC_Mode flags
    static constexpr uint8_t BOX_ANTIGRAVITY = BOX_ID_FLIGHTMODE_COUNT;
    static constexpr uint8_t BOX_HEADADJ = 12;
    static constexpr uint8_t BOX_CAMSTAB = 13;
    static constexpr uint8_t BOX_BEEPER_ON = 14;
    static constexpr uint8_t BOX_LED_LOW = 15;
    static constexpr uint8_t BOX_CALIBRATE = 16;
    static constexpr uint8_t BOX_OSD = 17;
    static constexpr uint8_t BOX_TELEMETRY = 18;
    static constexpr uint8_t BOX_SERVO1 = 19;
    static constexpr uint8_t BOX_SERVO2 = 20;
    static constexpr uint8_t BOX_SERVO3 = 21;
    static constexpr uint8_t BOX_BLACKBOX = 22;
    static constexpr uint8_t BOX_AIRMODE = 23;
    static constexpr uint8_t BOX_3D = 24;
    static constexpr uint8_t BOX_FPV_ANGLE_MIX = 25;
    static constexpr uint8_t BOX_BLACKBOX_ERASE = 26;
    static constexpr uint8_t BOX_CAMERA1 = 27;
    static constexpr uint8_t BOX_CAMERA2 = 28;
    static constexpr uint8_t BOX_CAMERA3 = 29;
    static constexpr uint8_t BOX_CRASH_FLIP = 30;
    static constexpr uint8_t BOX_PREARM = 31;
    static constexpr uint8_t BOX_BEEP_GPS_COUNT= 32;
    static constexpr uint8_t BOX_VTX_PIT_MODE = 33;
    static constexpr uint8_t BOX_PARALYZE = 34;
    static constexpr uint8_t BOX_USER1 = 35;
    static constexpr uint8_t BOX_USER2 = 36;
    static constexpr uint8_t BOX_USER3 = 37;
    static constexpr uint8_t BOX_USER4 = 38;
    static constexpr uint8_t BOX_PID_AUDIO = 39;
    static constexpr uint8_t BOX_ACRO_TRAINER = 40;
    static constexpr uint8_t BOX_VTX_CONTROL_DISABLE = 41;
    static constexpr uint8_t BOX_LAUNCH_CONTROL = 42;
    static constexpr uint8_t BOX_MSP_OVERRIDE = 43;
    static constexpr uint8_t BOX_STICK_COMMAND_DISABLE = 44;
    static constexpr uint8_t BOX_BEEPER_MUTE = 45;
    static constexpr uint8_t BOX_READY = 46;
    static constexpr uint8_t BOX_LAP_TIMER_RESET = 47;
    static constexpr uint8_t BOX_COUNT = 48; // number of boxes

    typedef std::bitset<BOX_COUNT> bitset_t;
    typedef int serializeBoxFn(StreamBufWriter& dst, const box_t* box);
public:
    static const box_t* findBoxByBoxId(uint8_t boxId);
    static const box_t* findBoxByPermanentId(uint8_t permanent_id);

    static int serialize_box_name(StreamBufWriter& dst, const box_t* box);
    void serialize_box_reply_box_name(StreamBufWriter& dst, size_t page) const;
    void serialize_box_reply_permanent_id(StreamBufWriter& dst, size_t page) const;
    bool get_active_box_id(uint8_t boxId) const;
    void set_active_box_id(uint8_t boxId);
    void reset_active_box_id(uint8_t boxId);
protected:
    std::bitset<BOX_COUNT> _activeBoxIds {};
    // permanent IDs must uniquely identify BOX meaning, DO NOT REUSE THEM!
    static constexpr std::array<box_t, BOX_COUNT> boxes = {{
        { .id = BOX_ARM,         .permanent_id = 0,  .name = "ARM" },
        { .id = BOX_ANGLE,       .permanent_id = 1,  .name = "ANGLE" },
        { .id = BOX_HORIZON,     .permanent_id = 2,  .name = "HORIZON" },
        { .id = BOX_ALTITUDE_HOLD, .permanent_id = 3, .name = "ALTHOLD" },
        { .id = BOX_ANTIGRAVITY, .permanent_id = 4,  .name = "ANTI GRAVITY" },
        { .id = BOX_MAG,         .permanent_id = 5,  .name = "MAG" },
        { .id = BOX_HEADFREE,    .permanent_id = 6,  .name = "HEADFREE" },
        { .id = BOX_HEADADJ,     .permanent_id = 7,  .name = "HEADADJ" },
        { .id = BOX_CAMSTAB,     .permanent_id = 8,  .name = "CAMSTAB" },
//      { .id = BOX_CAM_TRIG,    .permanent_id = 9,  .name = "CAM_TRIG", },
//      { .id = BOX_GPS_HOME,    .permanent_id = 10, .name = "GPS HOME" },
        { .id = BOX_POSITION_HOLD, .permanent_id = 11, .name = "POS HOLD" },
        { .id = BOX_PASSTHRU,    .permanent_id = 12, .name = "PASSTHRU" },
        { .id = BOX_BEEPER_ON,   .permanent_id = 13, .name = "BEEPER" },
//      { .id = BOX_LED_MAX,     .permanent_id = 14, .name = "LEDMAX" }, (removed)
        { .id = BOX_LED_LOW,     .permanent_id = 15, .name = "LEDLOW" },
//      { .id = BOX_LLIGHTS,     .permanent_id = 16, .name = "LLIGHTS" }, (removed)
        { .id = BOX_CALIBRATE,   .permanent_id = 17, .name = "CALIBRATE" },
//      { .id = BOX_GOVERNOR,    .permanent_id = 18, .name = "GOVERNOR" }, (removed)
        { .id = BOX_OSD,         .permanent_id = 19, .name = "OSD DISABLE" },
        { .id = BOX_TELEMETRY,   .permanent_id = 20, .name = "TELEMETRY" },
//      { .id = BOX_GTUNE,       .permanent_id = 21, .name = "GTUNE" }, (removed)
//      { .id = BOX_RANGEFINDER, .permanent_id = 22, .name = "RANGEFINDER" }, (removed)
        { .id = BOX_SERVO1,      .permanent_id = 23, .name = "SERVO1" },
        { .id = BOX_SERVO2,      .permanent_id = 24, .name = "SERVO2" },
        { .id = BOX_SERVO3,      .permanent_id = 25, .name = "SERVO3" },
        { .id = BOX_BLACKBOX,    .permanent_id = 26, .name = "BLACKBOX_" },
        { .id = BOX_FAILSAFE,    .permanent_id = 27, .name = "FAILSAFE" },
        { .id = BOX_AIRMODE,     .permanent_id = 28, .name = "AIR MODE" },
        { .id = BOX_3D,          .permanent_id = 29, .name = "3D DISABLE / SWITCH" },
        { .id = BOX_FPV_ANGLE_MIX, .permanent_id = 30, .name = "FPV ANGLE MIX" },
        { .id = BOX_BLACKBOX_ERASE, .permanent_id = 31, .name = "BLACKBOX_ ERASE" },
        { .id = BOX_CAMERA1,     .permanent_id = 32, .name = "CAMERA CONTROL 1" },
        { .id = BOX_CAMERA2,     .permanent_id = 33, .name = "CAMERA CONTROL 2" },
        { .id = BOX_CAMERA3,     .permanent_id = 34, .name = "CAMERA CONTROL 3" },
        { .id = BOX_CRASH_FLIP,  .permanent_id = 35, .name = "FLIP OVER AFTER CRASH" },
        { .id = BOX_PREARM,      .permanent_id = 36, .name = "PREARM" },
        { .id = BOX_BEEP_GPS_COUNT, .permanent_id = 37, .name = "GPS BEEP SATELLITE COUNT" },
//      { .id = BOX3D_ON_A_SWITCH,.permanent_id= 38, .name = "3D ON A SWITCH", }, (removed)
        { .id = BOX_VTX_PIT_MODE, .permanent_id = 39, .name = "VTX PIT MODE" },
        { .id = BOX_USER1,       .permanent_id = 40, .name = "USER1" }, // may be overridden
        { .id = BOX_USER2,       .permanent_id = 41, .name = "USER2" },
        { .id = BOX_USER3,       .permanent_id = 42, .name = "USER3" },
        { .id = BOX_USER4,       .permanent_id = 43, .name = "USER4" },
        { .id = BOX_PID_AUDIO,   .permanent_id = 44, .name = "PID AUDIO" },
        { .id = BOX_PARALYZE,    .permanent_id = 45, .name = "PARALYZE" },
        { .id = BOX_GPS_RESCUE,  .permanent_id = 46, .name = "GPS RESCUE" },
        { .id = BOX_ACRO_TRAINER, .permanent_id = 47, .name = "ACRO TRAINER" },
        { .id = BOX_VTX_CONTROL_DISABLE, .permanent_id = 48, .name = "VTX CONTROL DISABLE" },
        { .id = BOX_LAUNCH_CONTROL, .permanent_id = 49, .name = "LAUNCH CONTROL" },
        { .id = BOX_MSP_OVERRIDE, .permanent_id = 50, .name = "MSP OVERRIDE" },
        { .id = BOX_STICK_COMMAND_DISABLE, .permanent_id = 51, .name = "STICK COMMANDS DISABLE" },
        { .id = BOX_BEEPER_MUTE, .permanent_id = 52, .name = "BEEPER MUTE" },
        { .id = BOX_READY,       .permanent_id = 53, .name = "READY" },
        { .id = BOX_LAP_TIMER_RESET, .permanent_id = 54, .name = "LAP TIMER RESET" },
    }};
};
