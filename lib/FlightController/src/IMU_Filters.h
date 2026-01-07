#pragma once

#include "DynamicNotchFilter.h"

#include <IMU_FiltersBase.h>
#include <RPM_Filters.h>

class Debug;
class MotorMixerBase;
class RPM_Filters;


class IMU_Filters : public IMU_FiltersBase {
public:
    // Filter parameters choosen to be compatible with MultiWii Serial Protocol MSP_FILTER_CONFIG and MSP_SET_FILTER_CONFIG
    struct config_t {
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
    enum { GYRO_LPF_MAX_HZ = 1000 }; // so little filtering above 1000Hz that to get less delay you might as well disable the filter
public:
    IMU_Filters(size_t motorCount, Debug& debug, float looptimeSeconds);
    IMU_Filters(size_t motorCount, Debug& debug, uint32_t looptime) = delete; // delete this overload so it is not accidentally called

    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;

    void setRPM_Filters(RPM_Filters* rpmFilters) { _rpmFilters = rpmFilters; }
    const RPM_Filters* getRPM_Filters() const { return _rpmFilters; }
    RPM_Filters* getRPM_Filters() { return _rpmFilters; }

    void setConfig(const config_t& config);
    const config_t& getConfig() const { return _config; }
    void setDynamicNotchFilterConfig(const DynamicNotchFilter::config_t& config);
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    const DynamicNotchFilter::config_t& getDynamicNotchFilterConfig() const { return _dynamicNotchFilter.getConfig(); }
#endif
protected:
    Debug& _debug;
    float _looptimeSeconds;
    size_t _motorCount;
    config_t _config {}; //!< configuration data is only changed in setConfig
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
    RPM_Filters* _rpmFilters {nullptr};
};
