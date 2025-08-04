#pragma once

#include <IMU_FiltersBase.h>
#include <cstdint>
#include <xyz_type.h>

class MotorMixerBase;
class RPM_Filter;


class IMU_Filters : public IMU_FiltersBase {
public:
    // Filter parameters choosen to be compatible with MultiWii Serial Protocol MSP_FILTER_CONFIG and MSP_SET_FILTER_CONFIG
    struct filters_t {
        enum { PT1 = 0, BIQUAD, PT2, PT3 }; // filter types
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
        uint8_t gyro_hardware_lpf;
        uint8_t rpm_filter_harmonics;
        uint8_t rpm_filter_min_hz;
    };
public:
    IMU_Filters(const MotorMixerBase& motorMixer, uint32_t looptimeUs);
    void setRPM_Filter(RPM_Filter* rpmFilter);
    void setFilterFromAHRS(bool filterFromAHRS) { _filterFromAHRS = filterFromAHRS; }
    void init(float Q);
public:
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;
    virtual void setFilters() override;
    void setFilters(const filters_t& filters);
    const filters_t& getFilters() const { return _filters; }
protected:
    const MotorMixerBase& _motorMixer;
    uint32_t _looptimeUs;
    size_t _motorCount;
    size_t _motorIndex {0};
    filters_t _filters {};
    uint32_t _filterFromAHRS {false};
    RPM_Filter* _rpmFilter {nullptr};
};
