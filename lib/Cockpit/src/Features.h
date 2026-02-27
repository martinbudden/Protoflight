#pragma once

#include <cstdint>


struct features_config_t {
    uint32_t enabledFeatures;
};

class Features {
public:
    static constexpr uint32_t FEATURE_RX_PPM              = (1U << 0U);
    static constexpr uint32_t FEATURE_INFLIGHT_ACC_CALIBRATE = (1U << 2U);
    static constexpr uint32_t FEATURE_RX_SERIAL           = (1U << 3U);
    static constexpr uint32_t FEATURE_MOTOR_STOP          = (1U << 4U);
    static constexpr uint32_t FEATURE_SERVO_TILT          = (1U << 5U);
    static constexpr uint32_t FEATURE_SOFTSERIAL          = (1U << 6U);
    static constexpr uint32_t FEATURE_GPS                 = (1U << 7U);
    static constexpr uint32_t FEATURE_RANGEFINDER         = (1U << 9U);
    static constexpr uint32_t FEATURE_TELEMETRY           = (1U << 10U);
    static constexpr uint32_t FEATURE_3D                  = (1U << 12U);
    static constexpr uint32_t FEATURE_RX_PARALLEL_PWM     = (1U << 13U);
    static constexpr uint32_t FEATURE_RX_MSP              = (1U << 14U);
    static constexpr uint32_t FEATURE_RSSI_ADC            = (1U << 15U);
    static constexpr uint32_t FEATURE_LED_STRIP           = (1U << 16U);
    static constexpr uint32_t FEATURE_DASHBOARD           = (1U << 17U);
    static constexpr uint32_t FEATURE_OSD                 = (1U << 18U);
    static constexpr uint32_t FEATURE_CHANNEL_FORWARDING  = (1U << 20U);
    static constexpr uint32_t FEATURE_TRANSPONDER         = (1U << 21U);
    static constexpr uint32_t FEATURE_AIRMODE             = (1U << 22U);
    static constexpr uint32_t FEATURE_RX_SPI              = (1U << 25U);
    //static constexpr uint32_t FEATURE_SOFTSPI           = (1U << 26U); (removed)
    static constexpr uint32_t FEATURE_ESC_SENSOR          = (1U << 27U);
    static constexpr uint32_t FEATURE_ANTI_GRAVITY        = (1U << 28U);
    //static constexpr uint32_t FEATURE_DYNAMIC_FILTER    = (1U << 29U); (removed)
public:
    bool isEnabled(uint32_t mask) const;
    void set(uint32_t mask);
    void clear(uint32_t mask);
    uint32_t enabledFeatures() const;
private:
    uint32_t _featureMask;
};
