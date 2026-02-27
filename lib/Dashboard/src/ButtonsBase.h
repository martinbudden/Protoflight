#pragma once

class FlightController;
class MotorMixerBase;
class ReceiverBase;
class ScreenBase;


class ButtonsBase {
public:
    virtual ~ButtonsBase() = default;
    ButtonsBase(ScreenBase* screen) :
        _screen(screen) {}
    virtual void update(FlightController& flightController, MotorMixerBase& motorMixer, const ReceiverBase& receiver) = 0;
protected:
    ScreenBase* _screen;
};
