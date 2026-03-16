#pragma once

#include <screen_base.h>
#include <sv_telemetry_data.h>

struct ahrs_data_t;


class ScreenM5 : public ScreenBase {
public:
    enum screen_size_e {
        SIZE_128x128,
        SIZE_80x160,
        SIZE_135x240,
        SIZE_320x240,
        SIZE_200x200,
        SIZE_540x960
    };
    enum mode_e { MODE_NORMAL = 1, MODE_INVERTED = 3, MODE_QRCODE = 5 };
public:
    ScreenM5();
private:
    // ScreenM5 is not copyable or moveable
    ScreenM5(const ScreenM5&) = delete;
    ScreenM5& operator=(const ScreenM5&) = delete;
    ScreenM5(ScreenM5&&) = delete;
    ScreenM5& operator=(ScreenM5&&) = delete;
public:
    inline screen_size_e get_screen_size() const { return _screen_size; }

    virtual void next_screen_mode() override;
    virtual void update(const AhrsMessageQueue& ahrs_message_queue, const MotorMixerBase& motor_mixer, const ReceiverBase& receiver) override;
    virtual void update_template(const ReceiverBase& receiver) override;
private:
    void set_screen_mode(mode_e screen_mode);
    inline mode_e get_screen_mode() const { return _screen_mode; }
    screen_size_e screen_size();

    void update_template128x128(const ReceiverBase& receiver) const;
    void update_received_data128x128(const ReceiverBase& _receiver) const;
    void update128x128(const TD_AHRS::data_t& ahrs_data, bool motorsIsOn) const; // M5Atom

    void update_template320x240(const ReceiverBase& receiver) const;
    void update_received_data320x240(const ReceiverBase& _receiver) const;
    void update320x240(const TD_AHRS::data_t& ahrs_data, bool motorsIsOn) const; // MCore

    void update_received_data(const ReceiverBase& _receiver);
    void update_ahrs_data(const ahrs_data_t& ahrs_data, bool motorsIsOn) const;

    static void display_eui(const char* prompt, const ReceiverBase::EUI_48_t& eui);
    static void display_eui_Compact(const char* prompt, const ReceiverBase::EUI_48_t& eui);
private:
    screen_size_e _screen_size {SIZE_320x240};
    mode_e _screen_mode {MODE_NORMAL};
    int _screen_rotation_offset {0};
};
