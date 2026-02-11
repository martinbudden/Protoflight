#pragma once

#include "ReceiverWatcher.h"
#include <ReceiverBase.h>

class AHRS;
class DisplayPortBase;
class FlightController;


class ScreenBase : public ReceiverWatcher {
public:
    virtual ~ScreenBase() = default;
    ScreenBase(const DisplayPortBase& displayPort) :
        _displayPort(displayPort)
    {
    }
public:
    virtual void nextScreenMode() = 0;
    virtual void update(const FlightController& flightController, const ReceiverBase& receiver) = 0;
    virtual void updateTemplate(const ReceiverBase& receiver) = 0;
    inline int getScreenSizeX() const { return _screenSizeX; }
    inline int getScreenSizeY() const { return _screenSizeY; }

    virtual void new_receiver_packet_available() override { _new_receiver_packet_available = true; }
protected:
    void updateScreenAndTemplate() {
        _templateIsUpdated = false;
        //update(ahrs, flightController, receiver); 
    }
    inline void clearnew_receiver_packet_available() { _new_receiver_packet_available = false; }
    inline bool is_new_receiver_packet_available() const { return _new_receiver_packet_available; }
protected:
    const DisplayPortBase& _displayPort;
    int _screenSizeX {};
    int _screenSizeY {};
    int _new_receiver_packet_available {};
    int _templateIsUpdated {false};
    int _remoteEUI_updated {false};
};
