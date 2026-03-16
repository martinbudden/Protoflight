#pragma once

#include "receiver_watcher.h"
#include <receiver_base.h>

class AhrsMessageQueue;
class FlightController;
class MotorMixerBase;


class ScreenBase : public ReceiverWatcher {
public:
    virtual ~ScreenBase() = default;
    ScreenBase() = default;
public:
    virtual void next_screen_mode() = 0;
    virtual void update(const AhrsMessageQueue& ahrs_message_queue, const MotorMixerBase& motor_mixer, const ReceiverBase& receiver) = 0;
    virtual void update_template(const ReceiverBase& receiver) = 0;
    inline int get_screen_size_x() const { return _screen_size_x; }
    inline int get_screen_size_y() const { return _screen_size_y; }

    virtual void new_receiver_packet_available() override { _new_receiver_packet_available = true; }
protected:
    void update_screen_and_template() {
        _template_is_updated = false;
        //update(ahrs, flight_controller, receiver);
    }
    inline void clearnew_receiver_packet_available() { _new_receiver_packet_available = false; }
    inline bool is_new_receiver_packet_available() const { return _new_receiver_packet_available; }
protected:
    int _screen_size_x {};
    int _screen_size_y {};
    int _new_receiver_packet_available {};
    int _template_is_updated {false};
    int _remote_eui_updated {false};
};
