#if defined(M5_UNIFIED)

#include "ScreenM5.h"

#include <AHRS.h>
#include <BlackboxMessageQueue.h>
#include <FlightController.h>
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

static constexpr float radiansToDegrees {180.0 / M_PI};

ScreenM5::ScreenM5(const AHRS& ahrs, const FlightController& flightController, const ReceiverBase& receiver) :
    ScreenBase(ahrs, flightController, receiver),
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

void ScreenM5::updateTemplate128x128() const
{
    M5.Lcd.setCursor(0, 10);
    displayEUI("M:", _receiver.getMyEUI());
    M5.Lcd.setCursor(0, 20);
    displayEUI("J:", _receiver.getPrimaryPeerEUI());

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

void ScreenM5::updateReceivedData128x128() const
{
    if (!_remoteEUI_updated) {
        M5.Lcd.setCursor(0, 20);
        displayEUI("J:", _receiver.getPrimaryPeerEUI());
    }

    // M5StickC
    int32_t yPos = 95;
    const ReceiverBase::controls_t controls = _receiver.getControls();

    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%7.3f", controls.throttle);
    M5.Lcd.setCursor(72, yPos);
    M5.Lcd.printf("%7.3f", controls.roll);

    yPos += 10;
    M5.Lcd.setCursor(12, yPos);
    M5.Lcd.printf("%7.3f", controls.yaw);
    M5.Lcd.setCursor(72, yPos);
    M5.Lcd.printf("%7.3f", controls.pitch);
}

void ScreenM5::update128x128(const TD_AHRS::data_t& ahrsData) const
{
    int32_t yPos = 35;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("r:%4.0f p:%4.0F y:%4.0F",  _flightController.getPitchAngleDegreesRaw(), _flightController.getPitchAngleDegreesRaw(), _flightController.getYawAngleDegreesRaw());

    yPos += 25;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("x:%4.0f y:%4.0f z:%4.0f", ahrsData.gyroRPS.x*radiansToDegrees, ahrsData.gyroRPS.y*radiansToDegrees, ahrsData.gyroRPS.z*radiansToDegrees);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("x:%4.1f y:%4.1f z:%4.1f", ahrsData.acc.x, ahrsData.acc.y, ahrsData.acc.z);

    M5.Lcd.setCursor(12, 118);
    M5.Lcd.print(_flightController.motorsIsOn() ? "ON " : "OFF");

    M5.Lcd.setCursor(72, 118);
    M5.Lcd.printf("%2d  ", static_cast<int>(_receiver.getTickCountDelta()));
}

void ScreenM5::updateTemplate320x240() const
{
    M5.Lcd.setCursor(0, 0);
    displayEUI("MAC:", _receiver.getMyEUI());

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

void ScreenM5::updateReceivedData320x240() const
{
    if (!_remoteEUI_updated) {
        M5.Lcd.setCursor(0, 20);
        displayEUI("REM:", _receiver.getPrimaryPeerEUI());
    }

    int32_t yPos = 115;

    M5.Lcd.setCursor(20, yPos);
    const ReceiverBase::controls_t controls = _receiver.getControls();
    M5.Lcd.printf("%8.4f", controls.throttle);
    M5.Lcd.setCursor(180, yPos);
    M5.Lcd.printf("%8.4f", controls.roll);

    yPos += 20;
    M5.Lcd.setCursor(20, yPos);
    M5.Lcd.printf("%8.4f", controls.yaw);
    M5.Lcd.setCursor(180, yPos);
    M5.Lcd.printf("%8.4f", controls.pitch);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    const uint32_t flipButton =_receiver.getSwitch(ReceiverAtomJoyStick::MOTOR_ON_OFF_SWITCH);
    const uint32_t mode = _receiver.getSwitch(ReceiverAtomJoyStick::MODE_SWITCH);
    const uint32_t altMode = _receiver.getSwitch(ReceiverAtomJoyStick::ALT_MODE_SWITCH);
    M5.Lcd.printf("M%1d %s A%1d F%1d ", static_cast<int>(mode), mode == ReceiverAtomJoyStick::MODE_STABLE ? "ST" : "SP", static_cast<int>(altMode), static_cast<int>(flipButton));

    M5.Lcd.setCursor(255, yPos);
    M5.Lcd.printf("%4d", static_cast<int>(_receiver.getTickCountDelta()));
}

void ScreenM5::update320x240(const TD_AHRS::data_t& ahrsData) const
{
    int32_t yPos = 45;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("pi:%5.0f ro:%5.0f ya:%5.0f", ahrsData.pitch, ahrsData.roll, ahrsData.yaw);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("gx:%5.0F gy:%5.0f gz:%5.0f", ahrsData.gyroRPS.x*radiansToDegrees, ahrsData.gyroRPS.y*radiansToDegrees, ahrsData.gyroRPS.z*radiansToDegrees);

    yPos += 20;
    M5.Lcd.setCursor(0, yPos);
    M5.Lcd.printf("ax:%5.2f ay:%5.2f az:%5.2f", ahrsData.acc.x, ahrsData.acc.y, ahrsData.acc.z);

    //const flight_controller_quadcopter_telemetry_t telemetry = _flightController.getTelemetryData();

    yPos = 220;
    M5.Lcd.setCursor(85, yPos);
    M5.Lcd.printf("%s", _flightController.motorsIsOn() ? "ON " : "OFF");
}
#pragma GCC diagnostic pop

void ScreenM5::updateTemplate()
{
    M5.Lcd.fillScreen(TFT_BLACK);

    switch (_screenSize) {
    case SIZE_128x128:
        updateTemplate128x128();
        break;
    case SIZE_135x240:
        [[fallthrough]];
    case SIZE_80x160:
        //updateTemplate80x160();
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        updateTemplate320x240();
    }
    _templateIsUpdated = true;
}

void ScreenM5::updateReceivedData()
{
    switch (_screenSize) {
    case SIZE_128x128:
        updateReceivedData128x128();
        break;
    case SIZE_135x240:
        [[fallthrough]];
    case SIZE_80x160:
        //updateReceivedData80x160();
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        updateReceivedData320x240();
    }
    _remoteEUI_updated = true;
}

void ScreenM5::updateAHRS_Data() const
{
    //const AHRS::data_t ahrsData {}; //!!= _ahrs.getAhrsDataForInstrumentationUsingLock();
    // need to get orientation from AHRS since flight controller does not update Euler angles when in rate mode
    //const Quaternion orientationENU {}; //!!= _ahrs.getOrientationForInstrumentationUsingLock();

    const AHRS::imu_data_t queueItem = _flightController.getBlackboxMessageQueue().getQueueItem();
    const Quaternion orientationENU = queueItem.orientation;

    const TD_AHRS::data_t tdAhrsData {
        //.pitch = _flightController.getPitchAngleDegreesRaw(),
        //.roll = _flightController.getRollAngleDegreesRaw(),
        //.yaw = _flightController.getYawAngleDegreesRaw(),
        // convert from ENU to NED
        .roll = orientationENU.calculateRollDegrees(),
        .pitch = -orientationENU.calculatePitchDegrees(),
        .yaw = orientationENU.calculateYawDegrees(),
        .gyroRPS = queueItem.accGyroRPS.gyroRPS,
        .acc = queueItem.accGyroRPS.acc,
        .gyroOffset = {},
        .accOffset = {}
    };
    switch (_screenSize) {
    case SIZE_128x128:
        update128x128(tdAhrsData);
        break;
    case SIZE_135x240:
        //update135x240(tdAhrsData);
        break;
    case SIZE_80x160:
        //update80x160(tdAhrsData);
        break;
    case SIZE_320x240:
        [[fallthrough]];
    default:
        update320x240(tdAhrsData);
        break;
    }
}

/*!
Update the screen with data from the AHRS and the receiver.
*/
void ScreenM5::update()
{
    // update the screen with the AHRS data
    if (_screenMode != ScreenM5::MODE_QRCODE) {
        // update the screen template if it hasn't been updated
//        const AHRS::data_t ahrsData = _ahrs.getAhrsDataForInstrumentationUsingLock();
        if (_templateIsUpdated == false) {
            updateTemplate();
        }
        updateAHRS_Data();
        if (isNewReceiverPacketAvailable()) {
            clearNewReceiverPacketAvailable();
            // update the screen with data received from the receiver
            updateReceivedData();
        }
    }
}
#endif // M5_UNIFIED
