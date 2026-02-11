#if defined(M5_UNIFIED)

#include "FlightController.h"
#include "ScreenM5.h"

#include <AHRS.h>
#include <AHRS_MessageQueue.h>
#include <DisplayPortBase.h>
#include <M5Unified.h>
#include <ReceiverAtomJoyStick.h>
#include <SV_TelemetryData.h>


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

ScreenM5::ScreenM5(const DisplayPortBase& displayPort) :
    ScreenBase(displayPort),
    _screenSize(screenSize()),
    _screenRotationOffset(
        (_screenSize == SIZE_80x160 || _screenSize == SIZE_135x240) ? 1 :
        _screenSize == SIZE_128x128 ? -1 :
         0)
{
    M5.Lcd.setRotation(_screenMode + _screenRotationOffset);
    M5.Lcd.setTextSize(_screenSize == ScreenM5::SIZE_128x128 || _screenSize == ScreenM5::SIZE_80x160 || _screenSize == ScreenM5::SIZE_135x240 ? 1 : 2);
    M5.Lcd.fillScreen(TFT_BLACK);
}

ScreenM5::screen_size_e ScreenM5::screenSize()
{
    M5.Lcd.setRotation(1); // set to default, to find screen size

    screen_size_e screenSize;
    switch (M5.Lcd.height()) {
    case SCREEN_HEIGHT_M5_ATOM_S3:
        screenSize = SIZE_128x128;
        _screenSizeX = 128;
        _screenSizeY = 128;
        break;
    case SCREEN_WIDTH_M5_STICK_C: // M5_STICK_C rotated by default
        screenSize = SIZE_80x160;
        _screenSizeX = 80;
        _screenSizeY = 160;
        break;
    case SCREEN_WIDTH_M5_STICK_C_PLUS: // M5_STICK_C_PLUS rotated by default
        screenSize = SIZE_135x240;
        _screenSizeX = 135;
        _screenSizeY = 240;
        break;
    case SCREEN_HEIGHT_M5_CORE:
        screenSize = SIZE_320x240;
        _screenSizeX = 320;
        _screenSizeY = 240;
        break;
    case SCREEN_HEIGHT_M5_CORE_INK:
        screenSize = SIZE_200x200;
        _screenSizeX = 200;
        _screenSizeY = 200;
        break;
    case SCREEN_HEIGHT_M5_PAPER:
        screenSize = SIZE_540x960;
        _screenSizeX = 5400;
        _screenSizeY = 9600;
        break;
    default:
        screenSize = SIZE_320x240;
        _screenSizeX = 320;
        _screenSizeY = 240;
        break;
    }
    return screenSize;
}

void ScreenM5::setScreenMode(ScreenM5::mode_e screenMode)
{
    _screenMode = screenMode;

    if (_screenMode == MODE_QRCODE) {
        M5.Lcd.setRotation(MODE_NORMAL + _screenRotationOffset);
        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.qrcode("https://github.com/martinbudden/protoflight", 0, 0, 128, 6);
    } else {
        M5.Lcd.setRotation(_screenMode + _screenRotationOffset);
        updateScreenAndTemplate();
    }
}

/*!
Cycles through the different screen modes.
*/
void ScreenM5::nextScreenMode()
{
    const mode_e screenMode =
        _screenMode == ScreenM5::MODE_NORMAL ? ScreenM5::MODE_INVERTED :
        _screenMode == ScreenM5::MODE_INVERTED ? ScreenM5::MODE_QRCODE :
        ScreenM5::MODE_NORMAL;
    setScreenMode(screenMode);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
/*!
Utility function to display a MAC address.
*/
void ScreenM5::displayEUI(const char* prompt, const ReceiverBase::EUI_48_t& eui)
{
    M5.Lcd.printf("%s%02X:%02X:%02X:%02X:%02X:%02X", prompt, eui.octets[0], eui.octets[1], eui.octets[2], eui.octets[3], eui.octets[4], eui.octets[5]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

/*!
Utility function to display a MAC address in a compact format, for devices with small screens.
*/
void ScreenM5::displayEUI_Compact(const char* prompt, const ReceiverBase::EUI_48_t& eui)
{
    M5.Lcd.printf("%s%02x%02x%02x:%02x%02x%02x", prompt, eui.octets[0], eui.octets[1], eui.octets[2], eui.octets[3], eui.octets[4], eui.octets[5]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

void ScreenM5::updateTemplate128x128(const ReceiverBase& receiver) const
{
    M5.Lcd.setCursor(0, 10);
    displayEUI("M:", receiver.get_my_eui());
    M5.Lcd.setCursor(0, 20);
    displayEUI("J:", receiver.get_primary_peer_eui());

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

void ScreenM5::updateReceivedData128x128(const ReceiverBase& receiver) const
{
    if (!_remoteEUI_updated) {
        M5.Lcd.setCursor(0, 20);
        displayEUI("J:", receiver.get_primary_peer_eui());
    }

    // M5StickC
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

void ScreenM5::update128x128(const TD_AHRS::data_t& ahrsData, bool motorsIsOn) const
{
    int32_t yPos = 35;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("r:%4.0f p:%4.0F y:%4.0F", ahrsData.pitch, ahrsData.roll, ahrsData.yaw);

    yPos += 25;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("x:%4.0f y:%4.0f z:%4.0f", ahrsData.gyroRPS.x*RADIANS_TO_DEGREES, ahrsData.gyroRPS.y*RADIANS_TO_DEGREES, ahrsData.gyroRPS.z*RADIANS_TO_DEGREES);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("x:%4.1f y:%4.1f z:%4.1f", ahrsData.acc.x, ahrsData.acc.y, ahrsData.acc.z);

    M5.Lcd.setCursor(12, 118);
    M5.Lcd.print(motorsIsOn ? "ON " : "OFF");

    //M5.Lcd.setCursor(72, 118);
    //M5.Lcd.printf("%2d  ", static_cast<int>(receiver.get_tick_count_delta()));
}

void ScreenM5::updateTemplate320x240(const ReceiverBase& receiver) const
{
    M5.Lcd.setCursor(0, 0);
    displayEUI("MAC:", receiver.get_my_eui());

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

void ScreenM5::updateReceivedData320x240(const ReceiverBase& receiver) const
{
    if (!_remoteEUI_updated) {
        M5.Lcd.setCursor(0, 20);
        displayEUI("REM:", receiver.get_primary_peer_eui());
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
    const uint32_t flipButton = receiver.get_switch(ReceiverAtomJoyStick::MOTOR_ON_OFF_SWITCH);
    const uint32_t mode = receiver.get_switch(ReceiverAtomJoyStick::MODE_SWITCH);
    const uint32_t altMode = receiver.get_switch(ReceiverAtomJoyStick::ALT_MODE_SWITCH);
    M5.Lcd.printf("M%1d %s A%1d F%1d ", static_cast<int>(mode), mode == ReceiverAtomJoyStick::MODE_STABLE ? "ST" : "SP", static_cast<int>(altMode), static_cast<int>(flipButton));

    M5.Lcd.setCursor(255, yPos);
    M5.Lcd.printf("%4d", static_cast<int>(receiver.get_tick_count_delta()));
}

void ScreenM5::update320x240(const TD_AHRS::data_t& ahrsData, bool motorsIsOn) const
{
    int32_t yPos = 45;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:%5.0f ro:%5.0f ya:%5.0f", ahrsData.pitch, ahrsData.roll, ahrsData.yaw);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("gx:%5.0F gy:%5.0f gz:%5.0f", ahrsData.gyroRPS.x*RADIANS_TO_DEGREES, ahrsData.gyroRPS.y*RADIANS_TO_DEGREES, ahrsData.gyroRPS.z*RADIANS_TO_DEGREES);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ax:%5.2f ay:%5.2f az:%5.2f", ahrsData.acc.x, ahrsData.acc.y, ahrsData.acc.z);

    //const flight_controller_quadcopter_telemetry_t telemetry = _flightController.getTelemetryData();

    yPos = 220;
    M5.Lcd.setCursor(85, yPos);
    M5.Lcd.printf("%s", motorsIsOn ? "ON " : "OFF");
}
#pragma GCC diagnostic pop

void ScreenM5::updateTemplate(const ReceiverBase& receiver)
{
    if (_displayPort.isGrabbed()) {
        return;
    }
    M5.Lcd.fillScreen(TFT_BLACK);

    switch (_screenSize) {
    case SIZE_128x128:
        updateTemplate128x128(receiver);
        break;
    case SIZE_135x240:
        [[fallthrough]];
    case SIZE_80x160:
        //updateTemplate80x160();
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        updateTemplate320x240(receiver);
    }
    _templateIsUpdated = true;
}

void ScreenM5::updateReceivedData(const ReceiverBase& receiver)
{
    switch (_screenSize) {
    case SIZE_128x128:
        updateReceivedData128x128(receiver);
        break;
    case SIZE_135x240:
        [[fallthrough]];
    case SIZE_80x160:
        //updateReceivedData80x160(receiver);
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        updateReceivedData320x240(receiver);
    }
    _remoteEUI_updated = true;
}

void ScreenM5::updateAHRS_Data(const ahrs_data_t& ahrsData, bool motorsIsOn) const
{
    const Quaternion orientation = ahrsData.orientation;

    const TD_AHRS::data_t tdAhrsData {
        .roll = FlightController::rollAngleDegreesNED(orientation),
        .pitch = FlightController::pitchAngleDegreesNED(orientation),
        .yaw = FlightController::yawAngleDegreesNED(orientation),
        .gyroRPS = ahrsData.accGyroRPS.gyroRPS,
        .acc = ahrsData.accGyroRPS.acc,
        .gyroOffset = {},
        .accOffset = {}
    };
    switch (_screenSize) {
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
void ScreenM5::update(const FlightController& flightController, const ReceiverBase& receiver)
{
    if (_displayPort.isGrabbed()) {
        return;
    }
    // update the screen with the AHRS data
    if (_screenMode != ScreenM5::MODE_QRCODE) {
        // update the screen template if it hasn't been updated
        if (_templateIsUpdated == false) {
            updateTemplate(receiver);
        }
        ahrs_data_t ahrsData;
        flightController.getAHRS_MessageQueue().PEEK_AHRS_DATA(ahrsData);
        const bool motorsIsOn = flightController.motorsIsOn();
        updateAHRS_Data(ahrsData, motorsIsOn);
        if (is_new_receiver_packet_available()) {
            clearnew_receiver_packet_available();
            // update the screen with data received from the receiver
            updateReceivedData(receiver);
        }
    }
}
#endif // M5_UNIFIED
