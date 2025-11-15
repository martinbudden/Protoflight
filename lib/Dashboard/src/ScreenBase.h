#pragma once

#include <ReceiverBase.h>

class AHRS;
class DisplayPortBase;
class FlightController;


class ScreenBase : public ReceiverWatcher {
public:
    virtual ~ScreenBase() = default;
    ScreenBase(const DisplayPortBase& displayPort, const AHRS& ahrs, const FlightController& flightController, const ReceiverBase& receiver) :
        _displayPort(displayPort), _ahrs(ahrs), _flightController(flightController), _receiver(receiver) {}
public:
    virtual void nextScreenMode() = 0;
    virtual void update() = 0;
    virtual void updateTemplate() = 0;
    inline int getScreenSizeX() const { return _screenSizeX; }
    inline int getScreenSizeY() const { return _screenSizeY; }

    virtual void newReceiverPacketAvailable() override { _newReceiverPacketAvailable = true; }
protected:
    void updateScreenAndTemplate() { _templateIsUpdated = false; update(); }
    inline void clearNewReceiverPacketAvailable() { _newReceiverPacketAvailable = false; }
    inline bool isNewReceiverPacketAvailable() const { return _newReceiverPacketAvailable; }
protected:
    const DisplayPortBase& _displayPort;
    const AHRS& _ahrs;
    const FlightController& _flightController;
    const ReceiverBase& _receiver;
    int _screenSizeX {};
    int _screenSizeY {};
    int _newReceiverPacketAvailable {};
    int _templateIsUpdated {false};
    int _remoteEUI_updated {false};
};
