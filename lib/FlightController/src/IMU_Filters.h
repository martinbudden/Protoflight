#pragma once

#include "DynamicNotchFilter.h"
#include "Targets.h"

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

class IMU_Filters : public ImuFiltersBase {
public:
    // Filter parameters choosen to be compatible with MultiWii Serial Protocol MSP_FILTER_CONFIG and MSP_SET_FILTER_CONFIG
    enum { GYRO_LPF_MAX_HZ = 1000 }; // so little filtering above 1000Hz that to get less delay you might as well disable the filter
public:
    IMU_Filters(float looptimeSeconds);
    IMU_Filters(uint32_t looptime) = delete; // delete this overload so it is not accidentally called

    virtual void filter(xyz_t& gyro_rps, xyz_t& acc, float deltaT, Debug& debug) override;

    void setRPM_Filters(RpmFilters* rpmFilters) { _rpmFilters = rpmFilters; }
    const RpmFilters* getRPM_Filters() const { return _rpmFilters; }
    RpmFilters* getRPM_FiltersMutable() { return _rpmFilters; }

    void setConfig(const imu_filters_config_t& config);
    const imu_filters_config_t& getConfig() const { return _config; }
    void set_dynamic_notch_filter_config(const dynamic_notch_filter_config_t& config);
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    const dynamic_notch_filter_config_t& get_dynamic_notch_filter_config() const { return _dynamicNotchFilter.getConfig(); }
#endif
protected:
    float _looptimeSeconds;
    imu_filters_config_t _config {}; //!< configuration data is only changed in setConfig
    bool  _useGyroNotch1 {false};
    bool  _useGyroNotch2 {false};

    FilterBaseT<xyz_t>* _gyroLPF1 {nullptr};

    PowerTransferFilter1T<xyz_t> _accLPF;
    PowerTransferFilter1T<xyz_t> _gyroLPF1_PT1;
#if defined(USE_GYRO_FILTERS_EXTENDED)
    PowerTransferFilter2T<xyz_t> _gyroLPF1_PT2;
    BiquadFilterT<xyz_t> _gyroLPF1_Biquad;
#endif
    PowerTransferFilter1T<xyz_t>  _gyroLPF2;
    BiquadFilterT<xyz_t> _gyroNotch1;
    BiquadFilterT<xyz_t> _gyroNotch2;
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    DynamicNotchFilter _dynamicNotchFilter;
#endif
    RpmFilters* _rpmFilters {nullptr};
};
