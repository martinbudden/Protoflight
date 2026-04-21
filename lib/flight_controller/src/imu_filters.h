#pragma once

#include "dynamic_notch_filter.h"
#include "targets.h"

#include <imu_filters_base.h>
#include <rpm_filters.h>

class Debug;
class MotorMixerBase;
class RpmFilters;


struct imu_filters_config_t {
    enum { PT1 = 0, BIQUAD, PT2, PT3, NOT_SET = 0xFF }; // filter types
    uint16_t acc_lpf_hz;
    uint16_t gyro_lpf1_hz;
    uint16_t gyro_lpf2_hz;
    uint16_t gyro_notch1_hz;
    uint16_t gyro_notch1_cutoff;
    uint16_t gyro_notch2_hz;
    uint16_t gyro_notch2_cutoff;
    //uint16_t gyro_dynamic_lpf1_min_hz;
    //uint16_t gyro_dynamic_lpf1_max_hz;
    uint8_t gyro_lpf1_type;
    uint8_t gyro_lpf2_type;
    // uint8_t gyro_hardware_lpf; // this ignored, this is set in the IMU driver
};

class ImuFilters : public ImuFiltersBase {
public:
    // Filter parameters choosen to be compatible with MultiWii Serial Protocol MSP_FILTER_CONFIG and MSP_SET_FILTER_CONFIG
    enum { GYRO_LPF_MAX_HZ = 1000 }; // so little filtering above 1000Hz that to get less delay you might as well disable the filter
public:
    ImuFilters(float looptime_seconds);
    ImuFilters(uint32_t looptime) = delete; // delete this overload so it is not accidentally called

    virtual void filter(xyz_t& acc, xyz_t& gyro_rps, float delta_t, Debug& debug) override;

    void set_rpm_filters(RpmFilters* rpm_filters) { _rpm_filters = rpm_filters; }
    const RpmFilters* get_rpm_filters() const { return _rpm_filters; }
    RpmFilters* get_rpm_filters_mutable() { return _rpm_filters; }

    void set_config(const imu_filters_config_t& config);
    const imu_filters_config_t& get_config() const { return _config; }
    void set_dynamic_notch_filter_config(const dynamic_notch_filter_config_t& config);
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    const dynamic_notch_filter_config_t& get_dynamic_notch_filter_config() const { return _dynamic_notch_filter.get_config(); }
#endif
protected:
    float _looptime_seconds;
    imu_filters_config_t _config {}; //!< configuration data is only changed in set_config
    bool  _use_gyro_notch1 {false};
    bool  _use_gyro_notch2 {false};

    FilterBaseT<xyz_t>* _gyro_lpf1 {nullptr};

    PowerTransferFilter1T<xyz_t> _acc_lpf;
    PowerTransferFilter1T<xyz_t> _gyro_lpf1_pt1;
#if defined(USE_GYRO_FILTERS_EXTENDED)
    PowerTransferFilter2T<xyz_t> _gyro_lpf1_pt2;
    BiquadFilterT<xyz_t> _gyro_lpf1_biquad;
#endif
    PowerTransferFilter1T<xyz_t>  _gyro_lpf2;
    BiquadFilterT<xyz_t> _gyro_notch1;
    BiquadFilterT<xyz_t> _gyro_notch2;
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    DynamicNotchFilter _dynamic_notch_filter;
#endif
    RpmFilters* _rpm_filters {nullptr};
};
