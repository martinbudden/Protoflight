#if defined(M5_UNIFIED)

#include "ButtonsM5.h"
#include "ScreenM5.h"

#include "FlightController.h"
#include <M5Unified.h>


ButtonsM5::ButtonsM5(ScreenBase* screen) :
    ButtonsBase(screen)
{
    const int screenSizeX = _screen->getScreenSizeX();
    _drawPosX = (screenSizeX == 128) ? 115 : screenSizeX - 20;
    const int screenSizeY = _screen->getScreenSizeY();
    _drawPosY = (screenSizeY == 128) ? 115 : screenSizeY - 20;
}

/*!
Handle any button presses.

1. BtnA turns the motors on or off.
*/
void ButtonsM5::update(FlightController& flightController, const ReceiverBase& receiver)
{
    M5.update();
    // M5 Atom has only BtnA so use double click of BtnA for cycling through screen modes
    if (M5.getBoard() ==lgfx::board_M5AtomS3 && M5.BtnA.wasDoubleClicked()) {
        _screen->nextScreenMode();
    }
    if (M5.BtnA.wasPressed()) {
        // BtnA turns the motors on or off
        if (flightController.motorsIsOn()) {
            flightController.motorsSwitchOff();
        } else {
            flightController.motorsSwitchOn();
        }
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.print('A');
    } else if (M5.BtnA.wasReleased()) {
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.printf("  ");
    }

    if (M5.BtnB.wasPressed()) {
        // BtnB initiates binding
        receiver.broadcast_my_eui();
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.print('B');
    } else if (M5.BtnB.wasReleased()) {
        M5.Lcd.setCursor(_drawPosX, _drawPosY);
        M5.Lcd.printf("  ");
    }

    if (M5.BtnC.wasPressed() || M5.BtnB.wasDoubleClicked()) {
        // BtnC cycles through the different screen modes
        _screen->nextScreenMode();
    }

    if (M5.BtnPWR.wasDoubleClicked()) {
        M5.Power.powerOff();
    }
}

#endif // M5_UNIFIED
