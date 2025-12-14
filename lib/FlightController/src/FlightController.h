#pragma once

#include "FlightControllerTelemetry.h"
#include "Targets.h"

#include <Filters.h>
#include <PIDF.h>
#include <VehicleControllerBase.h>

#include <array>
#include <string>
#include <xyz_type.h>

class AHRS;
class AHRS_MessageQueue;
class Debug;
class DynamicNotchFilter;
class MotorMixerBase;
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

The FlightController has methods that may be called at startup, and in the context of the VehicleController Task,
the Receiver Task, and the AHRs task. This means there is the possibility of race conditions. A number of measures
have been put in place to help avoid race conditions. In particular:
*/
class FlightController : public VehicleControllerBase {
public:
    virtual ~FlightController() = default;
    FlightController(uint32_t taskIntervalMicroseconds, uint32_t outputToMotorsDenominator, MotorMixerBase& motorMixer, AHRS_MessageQueue& ahrsMessageQueue, Debug& debug);
    const AHRS_MessageQueue& getAHRS_MessageQueue() const { return _ahrsMessageQueue; }
    AHRS_MessageQueue& getAHRS_MessageQueue() { return _ahrsMessageQueue; }
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
    };
    static constexpr int TIME_CHECKS_COUNT = 4;
    //enum { AXIS_COUNT = 3 }; //!< roll, pitch, and yaw axis count
    enum { RP_AXIS_COUNT = 2 }; //!< roll and pitch axis count
    enum flight_dynamics_index_e { FD_ROLL = 0, FD_PITCH = 1, FD_YAW = 2, RPY_AXIS_COUNT = 3 };
    enum pid_index_e {
        ROLL_RATE_DPS = 0,
        PITCH_RATE_DPS = 1,
        YAW_RATE_DPS = 2,
        ROLL_ANGLE_DEGREES = 3,
        PITCH_ANGLE_DEGREES = 4,
#if defined(USE_SIN_ANGLE_PIDS)
        ROLL_SIN_ANGLE = 5,
        PITCH_SIN_ANGLE = 6,
        PID_COUNT = 7,
#else
        PID_COUNT = 5,
#endif
        PID_BEGIN = 0
    };

    // Filter parameters choosen to be compatible with MultiWii Serial Protocol MSP_FILTER_CONFIG and MSP_SET_FILTER_CONFIG
    struct filters_config_t {
        enum { PT1 = 0, BIQUAD, PT2, PT3 };
        uint16_t dterm_lpf1_hz;
        uint16_t dterm_lpf2_hz;
#if defined(USE_DTERM_FILTERS_EXTENDED)
        uint16_t dterm_notch_hz;
        uint16_t dterm_notch_cutoff;
        uint16_t dterm_dynamic_lpf1_min_hz;
        uint16_t dterm_dynamic_lpf1_max_hz;
        uint8_t dterm_lpf1_type;
        uint8_t dterm_lpf2_type;
#endif
        uint16_t yaw_lpf_hz;
        uint16_t output_lpf_hz;
        uint8_t rc_smoothing_feedforward_cutoff;
    };
    struct flight_mode_config_t {
        uint8_t level_race_mode; // aka "NFE race mode": angle mode on roll, acro mode on pitch
    };
    enum tpa_mode_e { TPA_MODE_PD, TPA_MODE_D, TPA_MODE_PDS };
    struct tpa_config_t {
        uint8_t tpa_mode;
        uint8_t tpa_rate;
        uint16_t tpa_breakpoint;
        int8_t tpa_low_rate;
        uint8_t tpa_low_always;
        uint16_t tpa_low_breakpoint;
    };
    struct tpa_runtime_t {
        float breakpoint;
        float multiplier;
        float lowBreakpoint;
        float lowMultiplier;
    };
    struct anti_gravity_config_t {
        uint8_t cutoff_hz;
        uint8_t p_gain;
        uint8_t i_gain;
    };
    struct anti_gravity_runtime_t {
        float PGain;
        float IGain;
    };
    static constexpr float D_MAX_DEFAULT_ROLL = 40.0F;
    static constexpr float D_MAX_DEFAULT_PITCH = 46.0F;
    struct d_max_config_t {
        std::array<uint8_t, RP_AXIS_COUNT> d_max; // Maximum D value on each axis
        uint8_t d_max_gain;     // gain factor for amount of gyro / setpoint activity required to boost D
        uint8_t d_max_advance;  // percentage multiplier for setpoint
    };
    static constexpr float DMAX_GAIN_FACTOR = 0.00008F;
    static constexpr float DMAX_SETPOINT_GAIN_FACTOR = 0.00008F;
    static constexpr float DMAX_RANGE_HZ = 85.0F;
    static constexpr float DMAX_LOWPASS_HZ = 35.0F;
    struct d_max_runtime_t {
        float gyroGain;
        float setpointGain;
        std::array<float, RP_AXIS_COUNT> percent;
        std::array<uint8_t, RP_AXIS_COUNT> max;
    };
#if defined(USE_YAW_SPIN_RECOVERY)
    enum yaw_spin_recovery_mode_e { YAW_SPIN_RECOVERY_OFF = 0, YAW_SPIN_RECOVERY_ON, YAW_SPIN_RECOVERY_AUTO };
    struct yaw_spin_recovery_config_t {
        int16_t yaw_spin_threshold;
        uint8_t yaw_spin_recovery;
    };
    struct yaw_spin_recovery_runtime_t {
        // yaw spin recovery
        float recoveredRPS;
        float partiallyRecoveredRPS;
    };
#endif
#if defined(USE_CRASH_RECOVERY)
    enum crash_recovery_e { CRASH_RECOVERY_OFF = 0, CRASH_RECOVERY_ON, CRASH_RECOVERY_BEEP, CRASH_RECOVERY_DISARM };
    struct crash_recovery_config_t {
        uint16_t crash_dthreshold;          // dterm crash value
        uint16_t crash_gthreshold;          // gyro crash value
        uint16_t crash_setpoint_threshold;  // setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
        uint16_t crash_time;                // ms
        uint16_t crash_delay;               // ms
        uint16_t crash_limit_yaw;           // limits yaw error rate, so crashes don't cause huge throttle increase
        uint8_t crash_recovery_angle;       // degrees
        uint8_t crash_recovery_rate;        // degrees per second
        uint8_t crash_recovery;             // off, on, on and beeps when it is in crash recovery mode
    };
    struct crash_recovery_runtime_t {
        uint32_t timeLimitUs;
        uint32_t timeDelayUs;
        int32_t recoveryAngleDeciDegrees;
        float recoveryRate;
        float gyroThresholdDPS;
        float DtermThresholdDPSPS; // degrees per second per second
        float setpointThresholdDPS;
        float limitYaw;
    };
#endif
#if defined(USE_ITERM_RELAX)
    enum iterm_relax_e { ITERM_RELAX_OFF, ITERM_RELAX_ON };
    struct iterm_relax_config_t {
        uint8_t iterm_relax;        // Enable iterm suppression during stick input
        uint8_t iterm_relax_setpoint_threshold; // Full iterm suppression once setpoint has exceeded this value (degrees per second)
        uint8_t iterm_relax_cutoff; // Cutoff frequency used by low pass filter which predicts average response of the quad to setpoint
    };
    struct iterm_relax_runtime_t {
        float setpointThresholdDPS;
    };
#endif
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

    enum pid_tuning_mode_e {
        PID_TUNING_STANDARD = 0,
        PID_TUNING_SIMPLIFIED_RP,
        PID_TUNING_SIMPLIFIED_RPY,
        PID_TUNING_MODE_COUNT
    };
    struct simplified_pid_settings_t {
        uint16_t multiplier;
        uint16_t roll_pitch_ratio;
        uint16_t i_gain;
        uint16_t d_gain;
        uint16_t pi_gain;
        uint16_t pitch_pi_gain;
        uint16_t d_max_gain;
        uint16_t k_gain;
    };

public:
    static float applyDeadband(float value, float deadband) {
        return (std::fabs(value) < deadband) ? 0.0F : (value >= 0.0F) ? value - deadband : value + deadband;
    }

    bool motorsIsOn() const;
    void motorsSwitchOff();
    void motorsSwitchOn();
    bool motorsIsDisabled() const;
    //! Sets blackboxActive flag so that ahrsData messages are sent to the blackbox task
    void setBlackboxActive(bool isActive) { _sh.blackboxActive = isActive; }

    virtual uint32_t getOutputPowerTimeMicroseconds() const override; //!!TODO: is this still needed?

    inline control_mode_e getControlMode() const { return _fcC.controlMode; }
    void setControlMode(control_mode_e controlMode);

    pid_tuning_mode_e getPID_TuningMode() const { return _pidTuningMode; }
    void setPID_TuningMode(pid_tuning_mode_e pidTuningMode);

    const std::string& getPID_Name(pid_index_e pidIndex) const;

    inline const PIDF& getPID(pid_index_e pidIndex) const { return _sh.PIDS[pidIndex]; }

    PIDF_uint16_t getPID_Constants(pid_index_e pidIndex) const;
    void setPID_Constants(pid_index_e pidIndex, const PIDF_uint16_t& pid16);

    const simplified_pid_settings_t& getSimplifiedPID_Settings() const;
    void setSimplifiedPID_Settings(const simplified_pid_settings_t& simplifiedPID_Settings);

    virtual PIDF_uint16_t getPID_MSP(size_t index) const override;
    void setPID_P_MSP(pid_index_e pidIndex, uint16_t kp);
    void setPID_PD_MSP(pid_index_e pidIndex, uint16_t kp); // Set P and change D to preserve P/D ratio
    void setPID_I_MSP(pid_index_e pidIndex, uint16_t ki);
    void setPID_D_MSP(pid_index_e pidIndex, uint16_t kd);
    void setPID_S_MSP(pid_index_e pidIndex, uint16_t ks);
    void setPID_K_MSP(pid_index_e pidIndex, uint16_t kk);

    PIDF_error_t getPID_Error(size_t index) const override;
    float getPID_Setpoint(size_t index) const override;

    inline float getPID_Setpoint(pid_index_e pidIndex) const { return _sh.PIDS[pidIndex].getSetpoint(); }
    void setPID_Setpoint(pid_index_e pidIndex, float setpoint) { _sh.PIDS[pidIndex].setSetpoint(setpoint); }

    void switchPID_integrationOn() { for (auto& pid : _sh.PIDS) { pid.switchIntegrationOn();} }
    void switchPID_integrationOff() { for (auto& pid : _sh.PIDS) { pid.switchIntegrationOff();} }

    // Functions to calculate roll, pitch, and yaw rates in the NED coordinate frame, converting from gyroRPS in the ENU coordinate frame
    // Note that for NED, roll is about the y-axis and pitch is about the x-axis.
    static inline float rollRateNED_DPS(const xyz_t& gyroENU_RPS) { return gyroENU_RPS.y * FlightController::radiansToDegrees; }
    static inline float pitchRateNED_DPS(const xyz_t& gyroENU_RPS) { return gyroENU_RPS.x * FlightController::radiansToDegrees; }
    static inline float yawRateNED_DPS(const xyz_t& gyroENU_RPS) { return -gyroENU_RPS.z * FlightController::radiansToDegrees; }

    static inline float rollSinAngleNED(const Quaternion& orientation) { return orientation.sinPitchClipped(); } // sin(x-180) = -sin(x)
    static inline float rollCosAngleNED(const Quaternion& orientation) { return orientation.cosPitch(); }
    static inline float rollAngleDegreesNED(const Quaternion& orientation) {
        float ret = orientation.calculatePitchDegrees();
        if (ret <= -180.0F) { ret = -ret - 180.0F; }
        return ret;
    };

    static inline float pitchSinAngleNED(const Quaternion& orientation) { return orientation.sinRollClipped(); } // NOTE: this is cheaper to calculate than sinRoll
    static inline float pitchCosAngleNED(const Quaternion& orientation)  { return orientation.cosRoll(); }
    static inline float pitchAngleDegreesNED(const Quaternion& orientation) { return orientation.calculateRollDegrees(); };

    static inline float yawAngleDegreesNED(const Quaternion& orientation) { return -orientation.calculateYawDegrees(); };

    flight_controller_quadcopter_telemetry_t getTelemetryData() const;
    const MotorMixerBase& getMotorMixer() const { return _motorMixer; }
    MotorMixerBase& getMotorMixer() { return _motorMixer; }

    const Debug& getDebug() const { return _debug; }
    Debug& getDebug() { return _debug; }
    inline uint32_t getTimeChecksMicroseconds(size_t index) const { return _sh.timeChecksMicroseconds[index]; } //!< Instrumentation time checks

    void setMaxAngleRates(float maxRollRateDPS, float maxPitchRateDPS, float maxYawRateDPS);
    float getMaxRollAngleDegrees() const { return _maxRollAngleDegrees; }
    float getMaxPitchAngleDegrees() const { return _maxPitchAngleDegrees; }

    void setFiltersConfig(const filters_config_t& filtersConfig);
    const filters_config_t& getFiltersConfig() const { return _filtersConfig; }

    void setFlightModeConfig(const flight_mode_config_t& flightModeConfig);
    const flight_mode_config_t& getFlightModeConfig() const { return _flightModeConfig; }

    void setTPA_Config(const tpa_config_t& tpaConfig);
    const tpa_config_t& getTPA_Config() const { return _tpaConfig; }

    void setAntiGravityConfig(const anti_gravity_config_t& antiGravityConfig);
    const anti_gravity_config_t& getAntiGravityConfig() const { return _antiGravityConfig; }

#if defined(USE_D_MAX)
    void setDMaxConfig(const d_max_config_t& dMaxConfig);
    const d_max_config_t& getDMaxConfig() const { return _dMaxConfig; }
#endif

#if defined(USE_ITERM_RELAX)
    void setITermRelaxConfig(const iterm_relax_config_t& iTermRelaxConfig);
    const iterm_relax_config_t& getITermRelaxConfig() const { return _iTermRelaxConfig; }
#endif

#if defined(USE_YAW_SPIN_RECOVERY)
    void setYawSpinRecoveryConfig(const yaw_spin_recovery_config_t& yawSpinRecoveryConfig);
    const yaw_spin_recovery_config_t& getYawSpinRecoveryConfig() const { return _yawSpinRecoveryConfig; }
#endif

#if defined(USE_CRASH_RECOVERY)
    void setCrashRecoveryConfig(const crash_recovery_config_t& crashRecoveryConfig);
    const crash_recovery_config_t& getCrashRecoveryConfig() const { return _crashRecoveryConfig; }
#endif

public:
    [[noreturn]] static void Task(void* arg);
public:
    void detectCrashOrSpin();
    void setYawSpinThresholdDPS(float yawSpinThresholdDPS);
    void recoverFromYawSpin(const xyz_t& gyroRPS, float deltaT);

    void calculateDMaxMultipliers();
    void initializeSetpointFilters(float setpointDeltaT);
    void applyDynamicPID_AdjustmentsOnThrottleChange(float throttle, uint32_t tickCount);
    void updateSetpoints(const controls_t& controls);
    void updateRateSetpointsForAngleMode(const Quaternion& orientationENU, float deltaT);

    float calculateITermError(size_t axis, float measurement);
    virtual void updateOutputsUsingPIDs(const AHRS::ahrs_data_t& ahrsDataNED) override;
    virtual void outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem) override;

private:
    static constexpr float DEGREES_TO_RADIANS = 3.14159265358979323846F / 180.0F;
    MotorMixerBase& _motorMixer;
    AHRS_MessageQueue& _ahrsMessageQueue;
    Debug& _debug;
    DynamicNotchFilter* _dynamicNotchFilter {nullptr};
    const uint32_t _outputToMotorsDenominator;
    const uint32_t _sendBlackboxMessageDenominator {8};

    //!!TODO: some constants below need to be made configurable
    const bool _useQuaternionSpaceForAngleMode {false};
    // ground mode handling
    const float _takeOffThrottleThreshold {0.2F};
    const uint32_t _takeOffTickThreshold {1000};
    // other constants
    float _maxRollRateDPS {500.0F};
    float _maxPitchRateDPS {500.0F};
    float _maxYawRateDPS {500.0F};
    const float _maxRollAngleDegrees { 60.0F }; // used for angle mode
    const float _maxPitchAngleDegrees { 60.0F }; // used for angle mode

    //
    // configuration and runtime data is const once it has been set in set*Config()
    //
    pid_tuning_mode_e _pidTuningMode {PID_TUNING_STANDARD};
    filters_config_t _filtersConfig {};
    flight_mode_config_t _flightModeConfig {};
    tpa_config_t _tpaConfig {};
    tpa_runtime_t _tpa { 0.0F, 1.0F, 0.0F, 1.0F };
    anti_gravity_config_t _antiGravityConfig {};
    anti_gravity_runtime_t _antiGravity {};
#if defined(USE_D_MAX)
    d_max_config_t _dMaxConfig {};
    d_max_runtime_t _dMax {};
#endif
#if defined(USE_ITERM_RELAX)
    iterm_relax_config_t _iTermRelaxConfig = {.iterm_relax=ITERM_RELAX_ON, .iterm_relax_setpoint_threshold = 40, .iterm_relax_cutoff=15};
    iterm_relax_runtime_t _iTermRelax = { .setpointThresholdDPS = 40.0F };
#endif
#if defined(USE_YAW_SPIN_RECOVERY)
    yaw_spin_recovery_config_t _yawSpinRecoveryConfig {.yaw_spin_threshold=1950, .yaw_spin_recovery=YAW_SPIN_RECOVERY_OFF};
    yaw_spin_recovery_runtime_t _yawSpin = {.recoveredRPS=100.0F*DEGREES_TO_RADIANS,. partiallyRecoveredRPS=400.F*DEGREES_TO_RADIANS};
#endif
#if defined(USE_CRASH_RECOVERY)
    crash_recovery_config_t _crashRecoveryConfig {};
    crash_recovery_runtime_t _crash {};
#endif

    //
    // member data is divided into structs, according to which task may set that data
    // so, for example, only functions running in the context of the receiver task can set data using _rxM
    // this is to help avoid race conditions
    // data that can be set by more than one task is in the shared_t struct
    //
    struct fc_t {
        uint32_t outputToMixerCount {0};
        control_mode_e controlMode {CONTROL_MODE_RATE};
        std::array<PIDF::PIDF_t, PID_COUNT> pidConstants {}; //!< the PID constants as set by tuning
        simplified_pid_settings_t simplifiedPID_Settings {};
        std::array<float, RPY_AXIS_COUNT> outputs;
        std::array<PowerTransferFilter1, RPY_AXIS_COUNT> outputFilters;
    };
    struct rx_t {
        // setpoint timing
        uint32_t setpointTickCountPrevious {0};
        uint32_t setpointTickCountSum {0};
        enum { SETPOINT_TICKCOUNT_COUNTER_START = 100};
        uint32_t setpointTickCountCounter {SETPOINT_TICKCOUNT_COUNTER_START};
        float setpointDeltaT {};
        float throttlePrevious {0.0F};
        float iTermAccelerator {0.0F};
        float yawRateSetpointDPS {0.0F};
        uint32_t useAngleMode {false}; // cache, to avoid complex condition test in updateOutputsUsingPIDs
        float TPA {1.0F}; //!< Throttle PID Attenuation, reduces DTerm for large throttle values
#if defined(USE_ITERM_RELAX)
        std::array<float, RP_AXIS_COUNT> setpointLPs {};
        std::array<float, RP_AXIS_COUNT> setpointHPs {};
#endif
    }; // rx_t
    enum angle_mode_calculation_state_e { STATE_CALCULATE_ROLL, STATE_CALCULATE_PITCH };
    struct angle_mode_calculation_state_t {
        angle_mode_calculation_state_e state { STATE_CALCULATE_ROLL };
        float rollSinAngle {0.0F};
        float pitchSinAngle {0.0F};
    };
    struct ah_t {
        angle_mode_calculation_state_t amcs;
        uint32_t sendBlackboxMessageCount {0};
        std::array<float, RP_AXIS_COUNT> dMaxMultiplier {1.0F, 1.0F}; // used even if USE_D_MAX not defined
    }; // ah_t
    struct shared_t {
        uint32_t takeOffCountStart {0};
        bool groundMode {true}; //! When in ground mode (ie pre-takeoff mode), the PID I-terms are set to zero to avoid integral windup on the ground
        bool crashDetected {false};
        bool blackboxActive {false};
#if defined(USE_YAW_SPIN_RECOVERY)
        bool yawSpinRecovery {false};
        float yawSpinThresholdDPS {0.0F};
#endif
        float outputThrottle {0.0F}; // throttle value is scaled to the range [-1,0, 1.0]
        std::array<PIDF, PID_COUNT> PIDS {}; //!< PIDF controllers, with dynamically altered PID constants
        PowerTransferFilter2 antiGravityThrottleFilter {};
        std::array<PowerTransferFilter1, PID_COUNT> dTermFilters1 {};
        std::array<PowerTransferFilter1, PID_COUNT> dTermFilters2 {};
        std::array<PowerTransferFilter3, RP_AXIS_COUNT> setpointDerivativeFilters {};
#if defined(USE_D_MAX)
        std::array<PowerTransferFilter2, RP_AXIS_COUNT> dMaxRangeFilters {};
        std::array<PowerTransferFilter2, RP_AXIS_COUNT> dMaxLowpassFilters {};
#endif
#if defined(USE_ITERM_RELAX)
        std::array<PowerTransferFilter1, RP_AXIS_COUNT> iTermRelaxFilters {};
#endif
        // instrumentation data
        std::array<uint32_t, TIME_CHECKS_COUNT + 1> timeChecksMicroseconds {};
    }; // shared_t

    fc_t _fcM;          //!< MODIFIABLE partition of member data that CAN  be used in the context of the Flight Controller Task
    const fc_t& _fcC;   //!< CONSTANT   partition of member data that MUST be used outside the context of the Flight Controller Task
    rx_t _rxM;          //!< MODIFIABLE partition of member data that CAN  be used in the context of the Receiver Task
    const rx_t& _rxC;   //!< CONSTANT   partition of member data that MUST be used outside the context of the Receiver Task
    ah_t _ahM;          //!< MODIFIABLE partition of member data that CAN  be used in the context of the AHRS Task
    shared_t _sh;       //!< member data that is set in the context of more than one task

    // Betaflight compatible mixer output scale factor: scales roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
    static constexpr float MIXER_OUTPUT_SCALE_FACTOR = 0.001F;
    // Betaflight-compatible PID scale factors.
    static constexpr PIDF::PIDF_t _scaleFactors = {
        0.032029F,
        0.244381F,
        0.000529F,
        0.01F, // !!TODO: provisional value
        0.013754F
    };
public:
    //!Default PIDs. For compatibility these are the same values as used by Betaflight.
    static constexpr std::array<PIDF_uint16_t, PID_COUNT> DefaultPIDs = {{
#if true
        { 45, 0, 0, 0, 0 }, // roll rate
        { 47, 0, 0, 0, 0 }, // pitch rate
        { 45, 0, 0, 0, 0 }, // yaw rate
        { 50, 0, 0, 0, 0 }, // roll angle
        { 50, 0, 0, 0, 0 }, // pitch angle
#else
        { 45, 80, 30, 120, 0 }, // roll rate
        { 47, 84, 34, 125, 0 }, // pitch rate
        { 45, 80,  0, 120, 0 }, // yaw rate
        { 50, 75, 75,  50, 0 }, // roll angle
        { 50, 75, 75,  50, 0 }, // pitch angle
#endif
#if defined(USE_SIN_ANGLE_PIDS)
        { 50, 75, 75,  50, 0 }, // roll sin angle
        { 50, 75, 75,  50, 0 }, // pitch sin angle
#endif
    }};
};
