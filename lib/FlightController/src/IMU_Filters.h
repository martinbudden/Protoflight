#pragma once

#include <Filters.h>
#include <IMU_FiltersBase.h>
#include <array>
#include <cstdint>
#include <xyz_type.h>

class MotorMixerBase;
class RPM_Filters;


class IMU_Filters : public IMU_FiltersBase {
public:
    enum { X = 0, Y = 1, Z = 2, AXIS_COUNT = 3 };

    // Filter parameters choosen to be compatible with MultiWii Serial Protocol MSP_FILTER_CONFIG and MSP_SET_FILTER_CONFIG
    struct config_t {
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
        uint8_t gyro_hardware_lpf; // this ignored, this is set in the IMU driver
        uint8_t rpm_filter_harmonics;
        uint8_t rpm_filter_min_hz;
    };
public:
    IMU_Filters(const MotorMixerBase& motorMixer, float looptimeSeconds);
    void setRPM_Filters(RPM_Filters* rpmFilters);
    void setFilterFromAHRS(bool filterFromAHRS) { _filterFromAHRS = filterFromAHRS; }
    void init(float Q);
public:
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;
    virtual void setFilters() override;
    void setConfig(const config_t& config);
    const config_t& getConfig() const { return _config; }
protected:
    const MotorMixerBase& _motorMixer;
    float _looptimeSeconds;
    size_t _motorCount;
    size_t _motorIndex {0};
    config_t _config {};
    uint32_t _filterFromAHRS {false};
    RPM_Filters* _rpmFilters {nullptr};
    FilterNull _filterNull;
    std::array<FilterBase*, AXIS_COUNT> _gyroLPF1 {};
    std::array<FilterBase*, AXIS_COUNT> _gyroLPF2 {};
    std::array<FilterBase*, AXIS_COUNT> _gyroNotch1 {};
    std::array<FilterBase*, AXIS_COUNT> _gyroNotch2 {};

    std::array<PowerTransferFilter1, AXIS_COUNT> _lpf2PT1;
    std::array<PowerTransferFilter2, AXIS_COUNT> _lpf2PT2;
    std::array<BiquadFilter, AXIS_COUNT> _lpf2Biquad;
    std::array<BiquadFilter, AXIS_COUNT> _notch1Biquad;
};
