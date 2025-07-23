#pragma once

#include <ButtonsBase.h>


class ButtonsM5 : public ButtonsBase {
public:
    ButtonsM5(FlightController& flightController, const ReceiverBase& receiver, ScreenBase* screen);
    virtual void update() override;
private:
    // ButtonsM5 is not copyable or moveable
    ButtonsM5(const ButtonsM5&) = delete;
    ButtonsM5& operator=(const ButtonsM5&) = delete;
    ButtonsM5(ButtonsM5&&) = delete;
    ButtonsM5& operator=(ButtonsM5&&) = delete;
private:
    int _drawPosX;
    int _drawPosY;
};
