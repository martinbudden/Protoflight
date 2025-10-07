#pragma once

#include "DynamicNotchFilter.h"
#include "RPM_Filters.h"
#include <FilterTemplates.h>
#include <IMU_FiltersBase.h>
#include <cstdint>
#include <xyz_type.h>

class Debug;
class MotorMixerBase;
class RPM_Filters;


class IMU_Filters : public IMU_FiltersBase {
public:
    // Filter parameters choosen to be compatible with MultiWii Serial Protocol MSP_FILTER_CONFIG and MSP_SET_FILTER_CONFIG
    struct config_t {
        enum { PT1 = 0, BIQUAD, PT2, PT3, NOT_SET = 0xFF }; // filter types
        uint16_t gyro_notch1_hz;
        uint16_t gyro_notch1_cutoff;
        uint16_t gyro_notch2_hz;
        uint16_t gyro_notch2_cutoff;
        uint16_t gyro_lpf1_hz;
        uint16_t gyro_lpf2_hz;
        uint16_t gyro_dynamic_lpf1_min_hz;
        uint16_t gyro_dynamic_lpf1_max_hz;
        uint8_t gyro_lpf1_type;
        uint8_t gyro_lpf2_type;
        // uint8_t gyro_hardware_lpf; // this ignored, this is set in the IMU driver
    };
public:
    IMU_Filters(size_t motorCount, Debug& debug, float looptimeMicroseconds);
#if defined(USE_RPM_FILTERS)
    const RPM_Filters* getRPM_Filters() const { return &_rpmFilters; }
    RPM_Filters* getRPM_Filters() { return &_rpmFilters; }
#else
    const RPM_Filters* getRPM_Filters() const { return nullptr; }
    RPM_Filters* getRPM_Filters() { return nullptr; }
#endif
public:
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;
    virtual void setFilters() override;
    void setConfig(const config_t& config);
    const config_t& getConfig() const { return _config; }
    void setDynamicNotchFilterConfig(const DynamicNotchFilter::config_t& config);
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    const DynamicNotchFilter::config_t& getDynamicNotchFilterConfig() const { return _dynamicNotchFilter.getConfig(); }
#endif
    void setRPM_FiltersConfig(const RPM_Filters::config_t& config);
#if defined(USE_RPM_FILTERS)
    const RPM_Filters::config_t& getRPM_FiltersConfig() const { return _rpmFilters.getConfig(); }
#endif
protected:
    Debug& _debug;
    float _looptimeSeconds;
    size_t _motorCount;
    const config_t _config {}; //!< configuration data is const once it has been set in setConfig

    FilterBaseT<xyz_t>* _gyroLPF1 {nullptr};
    uint32_t  _useGyroNotch1 {false};
    uint32_t  _useGyroNotch2 {false};

    PowerTransferFilter1T<xyz_t> _gyroLPF1PT1;
    PowerTransferFilter2T<xyz_t> _gyroLPF1PT2;
    BiquadFilterT<xyz_t> _gyroLPF1Biquad;
    PowerTransferFilter1T<xyz_t>  _gyroLPF2;

    BiquadFilterT<xyz_t> _gyroNotch1;
    BiquadFilterT<xyz_t> _gyroNotch2;
#if defined(USE_DYNAMIC_NOTCH_FILTER)
    DynamicNotchFilter _dynamicNotchFilter;
#endif
#if defined(USE_RPM_FILTERS)
    RPM_Filters _rpmFilters;
#endif
};
