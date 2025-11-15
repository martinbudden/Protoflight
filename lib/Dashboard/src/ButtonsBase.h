#pragma once

class FlightController;
class ReceiverBase;
class ScreenBase;


class ButtonsBase {
public:
    virtual ~ButtonsBase() = default;
    ButtonsBase(FlightController& flightController, const ReceiverBase& receiver, ScreenBase* screen) :
        _flightController(flightController), _receiver(receiver), _screen(screen) {}
    virtual void update() = 0;
protected:
    FlightController& _flightController;
    const ReceiverBase& _receiver;
    ScreenBase* _screen;
};
