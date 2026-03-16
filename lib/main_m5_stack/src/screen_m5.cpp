#if defined(M5_UNIFIED)

#include "flight_controller.h"
#include "screen_m5.h"

#include <M5Unified.h>

#include <ahrs.h>
#include <ahrs_message_queue.h>
#include <display_port_base.h>
#include <motor_mixer_base.h>
#include <receiver_atom_joystick.h>
#include <sv_telemetry_data.h>


enum {
    SCREEN_WIDTH_M5_ATOM_S3 = 128,
    SCREEN_WIDTH_M5_STICK_C = 80,
    SCREEN_WIDTH_M5_STICK_C_PLUS = 135,
    SCREEN_WIDTH_M5_CORE = 320,
    SCREEN_WIDTH_M5_CORE_INK = 200,
    SCREEN_WIDTH_M5_PAPER=540
};

enum {
    SCREEN_HEIGHT_M5_ATOM_S3 = 128,
    SCREEN_HEIGHT_M5_STICK_C = 80,
    SCREEN_HEIGHT_M5_STICK_C_PLUS = 135,
    SCREEN_HEIGHT_M5_CORE = 240,
    SCREEN_HEIGHT_M5_CORE_INK = 200,
    SCREEN_HEIGHT_M5_PAPER=960
};

static constexpr float RADIANS_TO_DEGREES {180.0 / M_PI};

ScreenM5::ScreenM5() :
    _screen_size(screen_size()),
    _screen_rotation_offset(
        (_screen_size == SIZE_80x160 || _screen_size == SIZE_135x240) ? 1 :
        _screen_size == SIZE_128x128 ? -1 :
         0)
{
    M5.Lcd.setRotation(_screen_mode + _screen_rotation_offset);
    M5.Lcd.setTextSize(_screen_size == ScreenM5::SIZE_128x128 || _screen_size == ScreenM5::SIZE_80x160 || _screen_size == ScreenM5::SIZE_135x240 ? 1 : 2);
    M5.Lcd.fillScreen(TFT_BLACK);
}

ScreenM5::screen_size_e ScreenM5::screen_size()
{
    M5.Lcd.setRotation(1); // set to default, to find screen size

    screen_size_e screen_size;
    switch (M5.Lcd.height()) {
    case SCREEN_HEIGHT_M5_ATOM_S3:
        screen_size = SIZE_128x128;
        _screen_size_x = 128;
        _screen_size_y = 128;
        break;
    case SCREEN_WIDTH_M5_STICK_C: // M5_STICK_C rotated by default
        screen_size = SIZE_80x160;
        _screen_size_x = 80;
        _screen_size_y = 160;
        break;
    case SCREEN_WIDTH_M5_STICK_C_PLUS: // M5_STICK_C_PLUS rotated by default
        screen_size = SIZE_135x240;
        _screen_size_x = 135;
        _screen_size_y = 240;
        break;
    case SCREEN_HEIGHT_M5_CORE:
        screen_size = SIZE_320x240;
        _screen_size_x = 320;
        _screen_size_y = 240;
        break;
    case SCREEN_HEIGHT_M5_CORE_INK:
        screen_size = SIZE_200x200;
        _screen_size_x = 200;
        _screen_size_y = 200;
        break;
    case SCREEN_HEIGHT_M5_PAPER:
        screen_size = SIZE_540x960;
        _screen_size_x = 5400;
        _screen_size_y = 9600;
        break;
    default:
        screen_size = SIZE_320x240;
        _screen_size_x = 320;
        _screen_size_y = 240;
        break;
    }
    return screen_size;
}

void ScreenM5::set_screen_mode(ScreenM5::mode_e screen_mode)
{
    _screen_mode = screen_mode;

    if (_screen_mode == MODE_QRCODE) {
        M5.Lcd.setRotation(MODE_NORMAL + _screen_rotation_offset);
        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.qrcode("https://github.com/martinbudden/protoflight", 0, 0, 128, 6);
    } else {
        M5.Lcd.setRotation(_screen_mode + _screen_rotation_offset);
        update_screen_and_template();
    }
}

/*!
Cycles through the different screen modes.
*/
void ScreenM5::next_screen_mode()
{
    const mode_e screen_mode =
        _screen_mode == ScreenM5::MODE_NORMAL ? ScreenM5::MODE_INVERTED :
        _screen_mode == ScreenM5::MODE_INVERTED ? ScreenM5::MODE_QRCODE :
        ScreenM5::MODE_NORMAL;
    set_screen_mode(screen_mode);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
/*!
Utility function to display a MAC address.
*/
void ScreenM5::display_eui(const char* prompt, const ReceiverBase::EUI_48_t& eui)
{
    M5.Lcd.printf("%s%02X:%02X:%02X:%02X:%02X:%02X", prompt, eui.octets[0], eui.octets[1], eui.octets[2], eui.octets[3], eui.octets[4], eui.octets[5]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

/*!
Utility function to display a MAC address in a compact format, for devices with small screens.
*/
void ScreenM5::display_eui_Compact(const char* prompt, const ReceiverBase::EUI_48_t& eui)
{
    M5.Lcd.printf("%s%02x%02x%02x:%02x%02x%02x", prompt, eui.octets[0], eui.octets[1], eui.octets[2], eui.octets[3], eui.octets[4], eui.octets[5]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

void ScreenM5::update_template128x128(const ReceiverBase& receiver) const
{
    M5.Lcd.setCursor(0, 10);
    display_eui("M:", receiver.get_my_eui());
    M5.Lcd.setCursor(0, 20);
    display_eui("J:", receiver.get_primary_peer_eui());

    int yPos = 50;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Gyro");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Acc");

    // joysticks
    yPos = 95;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("T:");
    M5.Lcd.setCursor(60, yPos);
    M5.Lcd.printf("R:");

    yPos += 10;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Y:");
    M5.Lcd.setCursor(60, yPos);
    M5.Lcd.printf("P:");

    M5.Lcd.setCursor(0, 118);
    M5.Lcd.printf("M:");

    M5.Lcd.setCursor(60, 118);
    M5.Lcd.printf("D:");
}

void ScreenM5::update_received_data128x128(const ReceiverBase& receiver) const
{
    if (!_remote_eui_updated) {
        M5.Lcd.setCursor(0, 20);
        display_eui("J:", receiver.get_primary_peer_eui());
    }

    // M5_stickC
    int32_t yPos = 95;
#if defined(USE_PWM)
    const char* format = "%5d";
    const receiver_controls_pwm_t controls = receiver.get_controls_pwm();
#else
    const char* format = "%7.3f";
    const receiver_controls_t controls = receiver.get_controls();
#endif
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf(format, controls.throttle);
    M5.Lcd.setCursor(72, yPos);
    M5.Lcd.printf(format, controls.roll);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf(format, controls.yaw);
    M5.Lcd.setCursor(72, yPos);
    M5.Lcd.printf(format, controls.pitch);
}

void ScreenM5::update128x128(const TD_AHRS::data_t& ahrs_data, bool motorsIsOn) const
{
    int32_t yPos = 35;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("r:%4.0f p:%4.0F y:%4.0F", ahrs_data.pitch, ahrs_data.roll, ahrs_data.yaw);

    yPos += 25;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("x:%4.0f y:%4.0f z:%4.0f", ahrs_data.gyro_rps.x*RADIANS_TO_DEGREES, ahrs_data.gyro_rps.y*RADIANS_TO_DEGREES, ahrs_data.gyro_rps.z*RADIANS_TO_DEGREES);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("x:%4.1f y:%4.1f z:%4.1f", ahrs_data.acc.x, ahrs_data.acc.y, ahrs_data.acc.z);

    M5.Lcd.setCursor(12, 118);
    M5.Lcd.print(motorsIsOn ? "ON " : "OFF");

    //M5.Lcd.setCursor(72, 118);
    //M5.Lcd.printf("%2d  ", static_cast<int>(receiver.get_tick_count_delta()));
}

void ScreenM5::update_template320x240(const ReceiverBase& receiver) const
{
    M5.Lcd.setCursor(0, 0);
    display_eui("MAC:", receiver.get_my_eui());

    int32_t yPos = 115;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("T:");
    M5.Lcd.setCursor(160, yPos);
    M5.Lcd.printf("R:");

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("Y:");
    M5.Lcd.setCursor(160, yPos);
    M5.Lcd.printf("P:");

    yPos += 20;
    M5.Lcd.setCursor(160, yPos);
    M5.Lcd.printf("Ticks");

    M5.Lcd.setCursor(0, 220);
    M5.Lcd.printf("Motors:");
}

void ScreenM5::update_received_data320x240(const ReceiverBase& receiver) const
{
    if (!_remote_eui_updated) {
        M5.Lcd.setCursor(0, 20);
        display_eui("REM:", receiver.get_primary_peer_eui());
    }

    int32_t yPos = 115;

    M5.Lcd.setCursor(20, yPos);
#define USE_PWM
#if defined(USE_PWM)
    const char* format = "%5d";
    const receiver_controls_pwm_t controls = receiver.get_controls_pwm();
#else
    const char* format = "%8.4f";
    const receiver_controls_t controls = receiver.getControls();
#endif
    M5.Lcd.printf(format, controls.throttle);
    M5.Lcd.setCursor(180, yPos);
    M5.Lcd.printf(format, controls.roll);

    yPos += 20;
    M5.Lcd.setCursor(20, yPos);
    M5.Lcd.printf(format, controls.yaw);
    M5.Lcd.setCursor(180, yPos);
    M5.Lcd.printf(format, controls.pitch);
    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    const uint32_t flipButton = receiver.get_switch(ReceiverAtomJoystick::MOTOR_ON_OFF_SWITCH);
    const uint32_t mode = receiver.get_switch(ReceiverAtomJoystick::MODE_SWITCH);
    const uint32_t altMode = receiver.get_switch(ReceiverAtomJoystick::ALT_MODE_SWITCH);
    M5.Lcd.printf("M%1d %s A%1d F%1d ", static_cast<int>(mode), mode == ReceiverAtomJoystick::MODE_STABLE ? "ST" : "SP", static_cast<int>(altMode), static_cast<int>(flipButton));

    M5.Lcd.setCursor(255, yPos);
    M5.Lcd.printf("%4d", static_cast<int>(receiver.get_tick_count_delta()));
}

void ScreenM5::update320x240(const TD_AHRS::data_t& ahrs_data, bool motorsIsOn) const
{
    int32_t yPos = 45;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:%5.0f ro:%5.0f ya:%5.0f", ahrs_data.pitch, ahrs_data.roll, ahrs_data.yaw);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("gx:%5.0F gy:%5.0f gz:%5.0f", ahrs_data.gyro_rps.x*RADIANS_TO_DEGREES, ahrs_data.gyro_rps.y*RADIANS_TO_DEGREES, ahrs_data.gyro_rps.z*RADIANS_TO_DEGREES);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ax:%5.2f ay:%5.2f az:%5.2f", ahrs_data.acc.x, ahrs_data.acc.y, ahrs_data.acc.z);

    //const flight_controller_quadcopter_telemetry_t telemetry = _flight_controller.get_telemetry_data();

    yPos = 220;
    M5.Lcd.setCursor(85, yPos);
    M5.Lcd.printf("%s", motorsIsOn ? "ON " : "OFF");
}
#pragma GCC diagnostic pop

void ScreenM5::update_template(const ReceiverBase& receiver)
{
    M5.Lcd.fillScreen(TFT_BLACK);

    switch (_screen_size) {
    case SIZE_128x128:
        update_template128x128(receiver);
        break;
    case SIZE_135x240:
        [[fallthrough]];
    case SIZE_80x160:
        //update_template80x160();
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        update_template320x240(receiver);
    }
    _template_is_updated = true;
}

void ScreenM5::update_received_data(const ReceiverBase& receiver)
{
    switch (_screen_size) {
    case SIZE_128x128:
        update_received_data128x128(receiver);
        break;
    case SIZE_135x240:
        [[fallthrough]];
    case SIZE_80x160:
        //update_received_data80x160(receiver);
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        update_received_data320x240(receiver);
    }
    _remote_eui_updated = true;
}

void ScreenM5::update_ahrs_data(const ahrs_data_t& ahrs_data, bool motorsIsOn) const
{
    const Quaternion orientation = ahrs_data.orientation;

    const TD_AHRS::data_t tdAhrsData {
        .roll = FlightController::roll_angle_degrees_ned(orientation),
        .pitch = FlightController::pitch_angle_degrees_ned(orientation),
        .yaw = FlightController::yaw_angle_degrees_ned(orientation),
        .gyro_rps = ahrs_data.acc_gyro_rps.gyro_rps,
        .acc = ahrs_data.acc_gyro_rps.acc,
        .gyro_offset = {},
        .acc_offset = {}
    };
    switch (_screen_size) {
    case SIZE_128x128:
        update128x128(tdAhrsData, motorsIsOn);
        break;
    case SIZE_135x240:
        //update135x240(tdAhrsData, motorsIsOn);
        break;
    case SIZE_80x160:
        //update80x160(tdAhrsData, motorsIsOn);
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        update320x240(tdAhrsData, motorsIsOn);
        break;
    }
}

/*!
Update the screen with data from the AHRS and the receiver.
*/
void ScreenM5::update(const AhrsMessageQueue& ahrs_message_queue, const MotorMixerBase& motor_mixer, const ReceiverBase& receiver)
{
    // update the screen with the AHRS data
    if (_screen_mode != ScreenM5::MODE_QRCODE) {
        // update the screen template if it hasn't been updated
        if (_template_is_updated == false) {
            update_template(receiver);
        }
        ahrs_data_t ahrs_data;
        ahrs_message_queue.PEEK_AHRS_DATA(ahrs_data);
        const bool motorsIsOn = motor_mixer.motors_is_on();
        update_ahrs_data(ahrs_data, motorsIsOn);
        if (is_new_receiver_packet_available()) {
            clearnew_receiver_packet_available();
            // update the screen with data received from the receiver
            update_received_data(receiver);
        }
    }
}
#endif // M5_UNIFIED
