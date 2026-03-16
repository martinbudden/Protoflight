#pragma once

#include <buttons_base.h>

class FlightController;
class ReceiverBase;


class ButtonsM5 : public ButtonsBase {
public:
    ButtonsM5(const ScreenBase* screen);
    virtual void update(FlightController& flight_controller, MotorMixerBase& motor_mixer, const ReceiverBase& receiver, ScreenBase* screen) override;
private:
    // ButtonsM5 is not copyable or moveable
    ButtonsM5(const ButtonsM5&) = delete;
    ButtonsM5& operator=(const ButtonsM5&) = delete;
    ButtonsM5(ButtonsM5&&) = delete;
    ButtonsM5& operator=(ButtonsM5&&) = delete;
private:
    int _draw_pos_x;
    int _draw_pos_y;
};
