#pragma once

#include <cstdint>


class Features {
public:
    enum  features_e {
        FEATURE_RX_PPM              = (1U << 0U),
        FEATURE_INFLIGHT_ACC_CALIBRATE = (1U << 2U),
        FEATURE_RX_SERIAL           = (1U << 3U),
        FEATURE_MOTOR_STOP          = (1U << 4U),
        FEATURE_SERVO_TILT          = (1U << 5U),
        FEATURE_SOFTSERIAL          = (1U << 6U),
        FEATURE_GPS                 = (1U << 7U),
        FEATURE_RANGEFINDER         = (1U << 9U),
        FEATURE_TELEMETRY           = (1U << 10U),
        FEATURE_3D                  = (1U << 12U),
        FEATURE_RX_PARALLEL_PWM     = (1U << 13U),
        FEATURE_RX_MSP              = (1U << 14U),
        FEATURE_RSSI_ADC            = (1U << 15U),
        FEATURE_LED_STRIP           = (1U << 16U),
        FEATURE_DASHBOARD           = (1U << 17U),
        FEATURE_OSD                 = (1U << 18U),
        FEATURE_CHANNEL_FORWARDING  = (1U << 20U),
        FEATURE_TRANSPONDER         = (1U << 21U),
        FEATURE_AIRMODE             = (1U << 22U),
        FEATURE_RX_SPI              = (1U << 25U),
        //FEATURE_SOFTSPI           = (1U << 26U), (removed)
        FEATURE_ESC_SENSOR          = (1U << 27U),
        FEATURE_ANTI_GRAVITY        = (1U << 28U),
        //FEATURE_DYNAMIC_FILTER    = (1U << 29U), (removed)
    };
    struct config_t {
        uint32_t enabledFeatures;
    };
public:
    bool isEnabled(uint32_t mask) const;
    void set(uint32_t mask);
    void clear(uint32_t mask);
    uint32_t enabledFeatures() const;
private:
    uint32_t _featureMask;
};
