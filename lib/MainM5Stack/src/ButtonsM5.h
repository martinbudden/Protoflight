#pragma once

#include <ButtonsBase.h>

class FlightController;
class ReceiverBase;


class ButtonsM5 : public ButtonsBase {
public:
    ButtonsM5(ScreenBase* screen);
    virtual void update(FlightController& flightController, const ReceiverBase& receiver) override;
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
