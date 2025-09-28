#pragma once

#define USE_D_MAX

#include "FlightControllerTelemetry.h"

#include <Filters.h>
#include <MotorMixerBase.h>
#include <PIDF.h>
#include <RadioControllerBase.h>
#include <VehicleControllerBase.h>
#include <array>
#include <string>
#include <xyz_type.h>

class AHRS;
class Blackbox;
class Debug;
class RadioControllerBase;
class Quaternion;

/*!
The flight controller which uses the North East Down (NED) coordinate frame.

The AHRS uses the East North Up (ENU) coordinate frame.
For ENU
positive pitch is nose down
positive roll is left side up
positive yaw is nose left

For NED
positive pitch is nose up
positive roll is left side up
positive yaw is nose right
*/
class FlightController : public VehicleControllerBase {
public:
    virtual ~FlightController() = default;
    FlightController(uint32_t taskDenominator, const AHRS& ahrs, MotorMixerBase& motorMixer, RadioControllerBase& radioController, Debug& debug);
private:
    // FlightController is not copyable or moveable
    FlightController(const FlightController&) = delete;
    FlightController& operator=(const FlightController&) = delete;
    FlightController(FlightController&&) = delete;
    FlightController& operator=(FlightController&&) = delete;
public:
    static constexpr float radiansToDegrees = static_cast<float>(180.0 / M_PI);
    enum control_mode_e {
        CONTROL_MODE_RATE = 0,
        CONTROL_MODE_ANGLE = 1,
        CONTROL_MODE_HORIZON = 2,
        CONTROL_MODE_ALTITUDE_HOLD = 3
    };
    enum { AXIS_COUNT = 3 }; //!< roll, pitch, and yaw axis count
    enum { RP_AXIS_COUNT = 2 }; //!< roll and pitch axis count
    enum flight_dynamics_index_e { FD_ROLL = 0, FD_PITCH = 1, FD_YAW = 2, FD_AXIS_COUNT = 3 };
    enum pid_index_e {
        ROLL_RATE_DPS = 0,
        PITCH_RATE_DPS = 1,
        YAW_RATE_DPS = 2,
        ROLL_ANGLE_DEGREES = 3,
        PITCH_ANGLE_DEGREES = 4,
        ROLL_SIN_ANGLE = 5,
        PITCH_SIN_ANGLE = 6,
        PID_COUNT = 7,
        PID_BEGIN = 0
    };
    enum arming_flag_e {
        ARMED = 0x01,
        WAS_EVER_ARMED = 0x02,
        WAS_ARMED_WITH_PREARM = 0x04
    };
    enum log2_flight_mode_flag_e {
        LOG2_ANGLE_MODE         = 0,
        LOG2_HORIZON_MODE       = 1,
        LOG2_MAG_MODE           = 2,
        LOG2_ALT_HOLD_MODE      = 3,
        LOG2_GPS_HOME_MODE      = 4,
        LOG2_GPS_HOLD_MODE      = 5,
        LOG2_HEADFREE_MODE      = 6,
        LOG2_UNUSED_MODE        = 7, // old autotune
        LOG2_PASSTHRU_MODE      = 8,
        LOG2_RANGEFINDER_MODE   = 9,
        LOG2_FAILSAFE_MODE      = 10,
        LOG2_GPS_RESCUE_MODE    = 11
    };
    enum flight_mode_flag_e {
        ANGLE_MODE      = (1U << LOG2_ANGLE_MODE),
        HORIZON_MODE    = (1U << LOG2_HORIZON_MODE),
        MAG_MODE        = (1U << LOG2_MAG_MODE),
        ALT_HOLD_MODE   = (1U << LOG2_ALT_HOLD_MODE),
        GPS_HOME_MODE   = (1U << LOG2_GPS_HOME_MODE),
        GPS_HOLD_MODE   = (1U << LOG2_GPS_HOLD_MODE),
        HEADFREE_MODE   = (1U << LOG2_HEADFREE_MODE),
        UNUSED_MODE     = (1U << LOG2_UNUSED_MODE), // old autotune
        PASSTHRU_MODE   = (1U << LOG2_PASSTHRU_MODE),
        RANGEFINDER_MODE= (1U << LOG2_RANGEFINDER_MODE),
        FAILSAFE_MODE   = (1U << LOG2_FAILSAFE_MODE),
        GPS_RESCUE_MODE = (1U << LOG2_GPS_RESCUE_MODE)
    };

    // Filter parameters choosen to be compatible with MultiWii Serial Protocol MSP_FILTER_CONFIG and MSP_SET_FILTER_CONFIG
    struct filters_config_t {
        enum { PT1 = 0, BIQUAD, PT2, PT3 };
        uint16_t dterm_lpf1_hz;
        uint16_t dterm_lpf2_hz;
        uint16_t dterm_notch_hz;
        uint16_t dterm_notch_cutoff;
        uint16_t dterm_dynamic_lpf1_min_hz;
        uint16_t dterm_dynamic_lpf1_max_hz;
        uint16_t yaw_lpf_hz;
        uint8_t dterm_lpf1_type;
        uint8_t dterm_lpf2_type;
        uint16_t output_lpf_hz;
    };
    enum tpa_mode_e { TPA_MODE_PD, TPA_MODE_D, TPA_MODE_PDS };
    struct tpa_config_t {
        uint8_t tpa_mode;
        uint8_t tpa_rate;
        uint16_t tpa_breakpoint;
    };
    struct anti_gravity_config_t {
        uint8_t cutoff_hz;
        uint8_t p_gain;
        uint8_t i_gain;
    };
    struct d_max_config_t {
        std::array<uint8_t, AXIS_COUNT> d_max; // Maximum D value on each axis
        uint8_t d_max_gain; // gain factor for amount of gyro / setpoint activity required to boost D
        uint8_t d_max_advance; // percentage multiplier for setpoint
    };
    struct controls_t {
        uint32_t tickCount;
        float throttleStick;
        float rollStickDPS;
        float pitchStickDPS;
        float yawStickDPS;
        float rollStickDegrees;
        float pitchStickDegrees;
        control_mode_e controlMode;
    };

    typedef std::array<PIDF::PIDF_t, PID_COUNT> pidf_array_t;
    typedef std::array<PIDF_uint16_t, PID_COUNT> pidf_uint16_array_t;

public:
    const AHRS& getAHRS() const { return _ahrs; }

    inline bool motorsIsOn() const { return _mixer.motorsIsOn(); }
    void motorsSwitchOff();
    void motorsSwitchOn();
    void motorsToggleOnOff();
    inline bool motorsIsDisabled() const { return _mixer.motorsIsDisabled(); }
    void setBlackbox(Blackbox& blackbox) { _blackbox = &blackbox; }

    inline control_mode_e getControlMode() const { return _fcC.controlMode; }
    void setControlMode(control_mode_e controlMode);

    bool isArmingFlagSet(arming_flag_e armingFlag) const;
    bool isFlightModeFlagSet(flight_mode_flag_e flightModeFlag) const;
    flight_mode_flag_e getFlightModeFlags() const { return ANGLE_MODE; } //!!TODO
    bool isRcModeActive(uint8_t rcMode) const;

    float getBatteryVoltage() const;
    float getAmperage() const;

    uint8_t getCurrentPidProfileIndex() const { return 0; }
    uint8_t getPidProfileCount() const { return 1; }
    uint8_t getCurrentControlRateProfileIndex() const { return 0; }

    virtual uint32_t getOutputPowerTimeMicroseconds() const override;

    const std::string& getPID_Name(pid_index_e pidIndex) const;

    inline const PIDF& getPID(pid_index_e pidIndex) const { return _sh.PIDS[pidIndex]; }
    PIDF_uint16_t getPID_Constants(pid_index_e pidIndex) const;
    void setPID_Constants(pid_index_e pidIndex, const PIDF_uint16_t& pid16);

    virtual PIDF_uint16_t getPID_MSP(size_t index) const override;
    void setPID_P_MSP(pid_index_e pidIndex, uint16_t kp);
    void setPID_I_MSP(pid_index_e pidIndex, uint16_t ki);
    void setPID_D_MSP(pid_index_e pidIndex, uint16_t kd);
    void setPID_F_MSP(pid_index_e pidIndex, uint16_t kf);
    void setPID_S_MSP(pid_index_e pidIndex, uint16_t ks);

    inline float getPID_Setpoint(pid_index_e pidIndex) const { return _sh.PIDS[pidIndex].getSetpoint(); }
    void setPID_Setpoint(pid_index_e pidIndex, float setpoint) { _sh.PIDS[pidIndex].setSetpoint(setpoint); }

    void switchPID_integrationOn() { for (auto& pid : _sh.PIDS) { pid.switchIntegrationOn();} }
    void switchPID_integrationOff() { for (auto& pid : _sh.PIDS) { pid.switchIntegrationOff();} }

    // Functions to calculate roll, pitch, and yaw rates in the NED coordinate frame, converting from gyroRPS in the ENU coordinate frame
    // Note that for NED, roll is about the y-axis and pitch is about the x-axis.
    static inline float rollRateNED_DPS(const xyz_t& gyroENU_RPS) { return gyroENU_RPS.y * radiansToDegrees; }
    static inline float pitchRateNED_DPS(const xyz_t& gyroENU_RPS) { return gyroENU_RPS.x * radiansToDegrees; }
    static inline float yawRateNED_DPS(const xyz_t& gyroENU_RPS) { return -gyroENU_RPS.z * radiansToDegrees; }

    flight_controller_quadcopter_telemetry_t getTelemetryData() const;
    const MotorMixerBase& getMixer() const { return _mixer; }
    float getMixerAdjustedThrottle() const { return _fcC.mixerAdjustedThrottle; }

    const Debug& getDebug() const { return _debug; }
    Debug& getDebug() { return _debug; }

    void setFiltersConfig(const filters_config_t& filtersConfig);
    const filters_config_t& getFiltersConfig() const { return _filtersConfig; }

    void setTPA_Config(const tpa_config_t& tpaConfig);
    const tpa_config_t& getTPA_Config() const { return _tpaConfig; }

    void setAntiGravityConfig(const anti_gravity_config_t& antiGravityConfig);
    const anti_gravity_config_t& getAntiGravityConfig() const { return _antiGravityConfig; }

    void setDMaxConfig(const d_max_config_t& dMaxConfig);
    const d_max_config_t& getDMaxConfig() const { return _dMaxConfig; }
public:
    [[noreturn]] static void Task(void* arg);
public:
    void detectCrashOrSpin();
    void setYawSpinThresholdDPS(float yawSpinThresholdDPS) { _sh.yawSpinThresholdDPS = yawSpinThresholdDPS; }
    void recoverFromYawSpin(const xyz_t& gyroENU_RPS, float deltaT);

    void calculateDMaxMultipliers();
    void applyDynamicPID_AdjustmentsOnThrottleChange(float throttle, uint32_t tickCount);
    void updateSetpoints(const controls_t& controls);
    void updateRateSetpointsForAngleMode(const Quaternion& orientationENU, float deltaT);

    virtual void updateOutputsUsingPIDs(const xyz_t& gyroENU_RPS, const xyz_t& accENU, const Quaternion& orientationENU, float deltaT) override;
    virtual void outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem) override;

private:
    static constexpr float degreesToRadians { static_cast<float>(M_PI) / 180.0F };
    MotorMixerBase& _mixer;
    RadioControllerBase& _radioController;
    Debug& _debug;
    Blackbox* _blackbox {nullptr};
    const uint32_t _taskDenominator;

    //!!TODO: constants below need to be made configurable
    const uint32_t _useAngleModeOnRollAcroModeOnPitch {false}; // used for "level race mode" aka "NFE race mode"
    // ground mode handling
    const float _takeOffThrottleThreshold {0.2F};
    const uint32_t _takeOffTickThreshold {1000};
    const uint32_t _angleModeUseQuaternionSpace {false};
    // yaw spin recovery
    const float _yawSpinRecoveredRPS { 100.0F * degreesToRadians };
    const float _yawSpinPartiallyRecoveredRPS { 400.F * degreesToRadians };
    // TPA
    const float _TPA_multiplier {0.0F};
    const float _TPA_breakpoint {0.6F};
    // flight dynamics scaling
    const float _maxRollRateDPS {500.0F};
    const float _maxPitchRateDPS {500.0F};
    const float _rollRateAtMaxPowerDPS {1000.0};
    const float _pitchRateAtMaxPowerDPS {1000.0};
    const float _yawRateAtMaxPowerDPS {1000.0};


    // configuration data is const once it has been set in set*Config()
    const filters_config_t _filtersConfig {};
    const tpa_config_t _tpaConfig {};
    const anti_gravity_config_t _antiGravityConfig {};
    const float _antiGravityIGain {};
    const float _antiGravityPGain {};
#ifdef USE_D_MAX
    d_max_config_t _dMaxConfig {};
    static constexpr float D_MAX_GAIN_FACTOR = 0.00008F;
    static constexpr float D_MAX_SETPOINT_GAIN_FACTOR = 0.00008F;
    enum { D_MAX_RANGE_HZ = 85, D_MAX_LOWPASS_HZ = 35 };
    float _dMaxGyroGain {};
    float _dMaxSetpointGain {};
    std::array<float, RP_AXIS_COUNT> _dMaxPercent {};
    std::array<uint8_t, RP_AXIS_COUNT> _dMax {};
#endif

    // member data is divided into structs, according to which task may set that data
    // so, for example,  only functions running in the context of the receiver task can set data using _rxM
    // this is to help avoid race conditions
    // data that can be set by more than one task is in the shared_t struct
    struct fc_t {
        uint32_t taskSignalledCount {0};
        control_mode_e controlMode {CONTROL_MODE_RATE};
        float mixerAdjustedThrottle {0.0F};
        std::array<PIDF::PIDF_t, PID_COUNT> pidConstants {}; //!< the PID constants as set by tuning
    };
    struct rx_t {
        // setpoint timing
        uint32_t setpointTickCountPrevious {0};
        uint32_t setpointTickCountSum {0};
        enum { SETPOINT_TICKCOUNT_COUNTER_START = 100};
        uint32_t setpointTickCountCounter {SETPOINT_TICKCOUNT_COUNTER_START};
        float setpointDeltaT {};
        float throttlePrevious {0.0F};
        float yawRateSetpointDPS {0.0F};
        uint32_t useAngleMode {false}; // cache, to avoid complex condition test in updateOutputsUsingPIDs
        // throttle value is scaled to the range [-1,0, 1.0]
        float TPA {1.0F}; //!< Throttle PID Attenuation, reduces DTerm for large throttle values
    };
    struct ah_t {
        float rollStickSinAngle {0.0F};
        float rollRateSetpointDPS {0.0F};
        float rollSinAngle {0.0F};
        float pitchStickSinAngle {0.0F};
        float pitchRateSetpointDPS {0.0F};
        float pitchSinAngle {0.0F};
        enum { STATE_CALCULATE_ROLL, STATE_CALCULATE_PITCH };
        uint32_t angleModeCalculationState { STATE_CALCULATE_ROLL };
        std::array<float, PID_COUNT> outputs {}; //<! PID outputs. These are stored since the output from one PID may be used as the input to another
        std::array<float, RP_AXIS_COUNT> dMaxMultiplier {1.0F, 1.0F};
    };
    struct shared_t {
        int groundMode {true}; //! When in ground mode (ie pre-takeoff mode), the PID I-terms are set to zero to avoid integral windup on the ground
        uint32_t takeOffCountStart {0};
        float yawSpinThresholdDPS {0.0F};
        uint32_t yawSpinRecovery { false };
        float outputThrottle {0.0F};
        std::array<PIDF, PID_COUNT> PIDS {}; //!< PIDF controllers, with dynamically altered PID values
        std::array<PowerTransferFilter1, YAW_RATE_DPS + 1> outputFilters;
        PowerTransferFilter2 antiGravityThrottleFilter {};
        // DTerm filters
        PowerTransferFilter1 rollRateDTermFilter {};
        PowerTransferFilter1 pitchRateDTermFilter {};
        PowerTransferFilter1 rollAngleDTermFilter {};
        PowerTransferFilter1 pitchAngleDTermFilter {};
#if defined(USE_D_MAX)
        std::array<PowerTransferFilter2, RP_AXIS_COUNT> dMaxRangeFilter {};
        std::array<PowerTransferFilter2, RP_AXIS_COUNT> dMaxLowpassFilter {};
#endif
    };

    fc_t _fcM; //!< MODIFIABLE partition of member data CAN be set in the context of the Flight Controller Task
    const fc_t& _fcC; //!< CONSTANT partition of member data that MUST be used outside the context of the Flight Controller Task
    rx_t _rxM; //!< MODIFIABLE partition of member data CAN be set in the context of the Receiver Task
    const rx_t& _rxC; //!< CONSTANT partition of member data that MUST be used outside the context of the Receiver Task
    ah_t _ahM; //!< MODIFIABLE partition of member data CAN be set in the context of the AHRS Task
    shared_t _sh; //!< member data that is set in the context of more than one task

    // Betaflight-compatible PID scale factors.
    static constexpr PIDF::PIDF_t _scaleFactors = {
        0.032029F,
        0.244381F,
        0.000529F,
        0.013754F,
        0.01F // !!TODO: provisional value
    };
};
