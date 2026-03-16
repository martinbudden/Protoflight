#if defined(M5_UNIFIED)

#include "buttons_m5.h"
#include "screen_m5.h"

#include "flight_controller.h"

#include <M5Unified.h>
#include <motor_mixer_base.h>


ButtonsM5::ButtonsM5(const ScreenBase* screen)
{
    const int screen_size_x = screen->get_screen_size_x();
    _draw_pos_x = (screen_size_x == 128) ? 115 : screen_size_x - 20;
    const int screen_size_y = screen->get_screen_size_y();
    _draw_pos_y = (screen_size_y == 128) ? 115 : screen_size_y - 20;
}

/*!
Handle any button presses.

1. BtnA turns the motors on or off.
*/
void ButtonsM5::update(FlightController& flight_controller, MotorMixerBase& motor_mixer, const ReceiverBase& receiver, ScreenBase* screen)
{
    M5.update();
    // M5 Atom has only BtnA so use double click of BtnA for cycling through screen modes
    if (M5.getBoard() ==lgfx::board_M5AtomS3 && M5.BtnA.wasDoubleClicked()) {
        screen->next_screen_mode();
    }
    if (M5.BtnA.wasPressed()) {
        // BtnA turns the motors on or off
        if (motor_mixer.motors_is_on()) {
            flight_controller.motors_switch_off(motor_mixer);
        } else {
            flight_controller.motors_switch_on(motor_mixer);
        }
        M5.Lcd.setCursor(_draw_pos_x, _draw_pos_y);
        M5.Lcd.print('A');
    } else if (M5.BtnA.wasReleased()) {
        M5.Lcd.setCursor(_draw_pos_x, _draw_pos_y);
        M5.Lcd.printf("  ");
    }

    if (M5.BtnB.wasPressed()) {
        // BtnB initiates binding
        receiver.broadcast_my_eui();
        M5.Lcd.setCursor(_draw_pos_x, _draw_pos_y);
        M5.Lcd.print('B');
    } else if (M5.BtnB.wasReleased()) {
        M5.Lcd.setCursor(_draw_pos_x, _draw_pos_y);
        M5.Lcd.printf("  ");
    }

    if (M5.BtnC.wasPressed() || M5.BtnB.wasDoubleClicked()) {
        // BtnC cycles through the different screen modes
        screen->next_screen_mode();
    }

    if (M5.BtnPWR.wasDoubleClicked()) {
        M5.Power.powerOff();
    }
}

#endif // M5_UNIFIED
