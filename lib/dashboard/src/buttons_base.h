#pragma once

class FlightController;
class MotorMixerBase;
class ReceiverBase;
class ScreenBase;


class ButtonsBase {
public:
    virtual ~ButtonsBase() = default;
    virtual void update(FlightController& flight_controller, MotorMixerBase& motor_mixer, const ReceiverBase& receiver, ScreenBase* screen) = 0;
};
