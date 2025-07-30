#pragma once

#include "FlightControllerTelemetry.h"
#include "MotorMixerBase.h"

#include <Filters.h>
#include <PIDF.h>
#include <RadioControllerBase.h>
#include <VehicleControllerBase.h>
#include <array>
#include <string>
#include <xyz_type.h>

class AHRS;
class Blackbox;
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
    FlightController(uint32_t taskIntervalMicroSeconds, const AHRS& ahrs, MotorMixerBase& motorMixer, RadioControllerBase& radioController);
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
    enum flight_mode_flag_e {
        ANGLE_MODE      = (1U << 0),
        HORIZON_MODE    = (1U << 1U),
        MAG_MODE        = (1U << 2U),
        ALT_HOLD_MODE   = (1U << 3U),
        GPS_HOME_MODE   = (1U << 4U),
        GPS_HOLD_MODE   = (1U << 5U),
        HEADFREE_MODE   = (1U << 6U),
        UNUSED_MODE     = (1U << 7U), // old autotune
        PASSTHRU_MODE   = (1U << 8U),
        RANGEFINDER_MODE= (1U << 9U),
        FAILSAFE_MODE   = (1U << 10U),
        GPS_RESCUE_MODE = (1U << 11U)
    };
    enum log2_flight_mode_flag_e { // must be kept in step with flight_mode_flag_e
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
    enum debug_mode_e {
        DEBUG_MODE_NONE,
        DEBUG_MODE_AHRS_TIMINGS
    };
    enum { DEBUG_VALUE_COUNT = 8 };

    // Filter parameters choosen to be compatible with MultiWii Serial Protocol MSP_FILTER_CONFIG and MSP_SET_FILTER_CONFIG
    struct filters_t {
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
    inline bool motorsIsOn() const { return _mixer.motorsIsOn(); }
    void motorsSwitchOff();
    void motorsSwitchOn();
    void motorsToggleOnOff();
    inline bool motorsIsDisabled() const { return _mixer.motorsIsDisabled(); }
    void setBlackbox(Blackbox& blackbox) { _blackbox = &blackbox; }

    inline control_mode_e getControlMode() const { return _controlMode; }
    void setControlMode(control_mode_e controlMode);

    bool isArmingFlagSet(arming_flag_e armingFlag) const;
    bool isFlightModeFlagSet(flight_mode_flag_e flightModeFlag) const;
    flight_mode_flag_e getFlightModeFlags() const { return ANGLE_MODE; } //!!TODO
    bool isRcModeActive(uint8_t rcMode) const;

    float getBatteryVoltage() const;
    float getAmperage() const;
    size_t getDebugValueCount() const;
    uint16_t getDebugValue(size_t index) const;

    uint8_t getCurrentPidProfileIndex() const { return 0; }
    uint8_t getPidProfileCount() const { return 1; }
    uint8_t getCurrentControlRateProfileIndex() const { return 0; }

    virtual uint32_t getOutputPowerTimeMicroSeconds() const override;

    const std::string& getPID_Name(pid_index_e pidIndex) const;

    inline const PIDF& getPID(pid_index_e pidIndex) const { return _PIDS[pidIndex]; }
    inline const PIDF::PIDF_t getPID_Constants(pid_index_e pidIndex) const { return _PIDS[pidIndex].getPID(); }
    void setPID_Constants(pid_index_e pidIndex, const PIDF::PIDF_t& pid);

    virtual PIDF_uint16_t getPID_MSP(size_t index) const override;
    void setPID_P_MSP(pid_index_e pidIndex, uint16_t kp) { _PIDS[pidIndex].setP(kp * _scaleFactors[pidIndex].kp); }
    void setPID_I_MSP(pid_index_e pidIndex, uint16_t ki) { _PIDS[pidIndex].setI(ki * _scaleFactors[pidIndex].ki); }
    void setPID_D_MSP(pid_index_e pidIndex, uint16_t kd) { _PIDS[pidIndex].setD(kd * _scaleFactors[pidIndex].kd); }
    void setPID_F_MSP(pid_index_e pidIndex, uint16_t kf) { _PIDS[pidIndex].setF(kf * _scaleFactors[pidIndex].kf); }

    inline float getPID_Setpoint(pid_index_e pidIndex) const { return _PIDS[pidIndex].getSetpoint(); }
    void setPID_Setpoint(pid_index_e pidIndex, float setpoint) { _PIDS[pidIndex].setSetpoint(setpoint); }
    void switchPID_integrationOn() { for (auto& pid : _PIDS) { pid.switchIntegrationOn();} }
    void switchPID_integrationOff() { for (auto& pid : _PIDS) { pid.switchIntegrationOff();} }

    // Functions to calculate roll, pitch, and yaw rates in the NED coordinate frame, converting from gyroRPS in the ENU coordinate frame
    // Note that for NED, roll is about the y-axis and pitch is about the x-axis.
    static inline float rollRateNED_DPS(const xyz_t& gyroENU_RPS) { return gyroENU_RPS.y * radiansToDegrees; }
    static inline float pitchRateNED_DPS(const xyz_t& gyroENU_RPS) { return gyroENU_RPS.x * radiansToDegrees; }
    static inline float yawRateNED_DPS(const xyz_t& gyroENU_RPS) { return -gyroENU_RPS.z * radiansToDegrees; }

    flight_controller_quadcopter_telemetry_t getTelemetryData() const;
    const filters_t& getFilters() const { return _filters; }
    void setFilters(const filters_t& filters);
    uint32_t getTaskIntervalMicroSeconds() const { return _taskIntervalMicroSeconds; }
    float getMixerThrottle() const { return _mixerThrottle; }
    debug_mode_e getDebugMode() const { return _debugMode; }
    void setDebugMode(debug_mode_e debugMode) { _debugMode = debugMode; }
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
public:
    void detectCrashOrSpin(uint32_t tickCount);
    void updateSetpoints(const controls_t& controls);
    void updateOutputsUsingPIDs(float deltaT);
    virtual void updateOutputsUsingPIDs(const xyz_t& gyroENU_RPS, const xyz_t& accENU, const Quaternion& orientationENU, float deltaT) override;
    void outputToMotors(float deltaT, uint32_t tickCount);
    virtual void loop(float deltaT, uint32_t tickCount) override;
private:
    MotorMixerBase& motorMixer(uint32_t taskIntervalMicroSeconds);
private:
    static constexpr float degreesToRadians { static_cast<float>(M_PI) / 180.0F };
    MotorMixerBase& _mixer;
    RadioControllerBase& _radioController;
    Blackbox* _blackbox {nullptr};
    control_mode_e _controlMode {CONTROL_MODE_RATE};
    uint32_t _useAngleMode {false}; // cache, to avoid complex condition test in updateOutputsUsingPIDs
    debug_mode_e _debugMode {DEBUG_MODE_AHRS_TIMINGS}; // default to providing AHRS timings to blackbox

    float _mixerThrottle {0.0F};

    // ground mode handling
    int _groundMode {true}; //! When in ground mode (ie pre-takeoff mode), the PID I-terms are set to zero to avoid integral windup on the ground
    float _takeOffThrottleThreshold {0.2F};
    uint32_t _takeOffTickCount {0};
    uint32_t _takeOffCountStart {0};
    uint32_t _takeOffTickThreshold {1000};

    // throttleStick value scaled to the range [-1,0, 1.0]
    float _throttleStick {0};
    float _TPA {1.0F}; //!< Throttle PID Attenuation, reduces DTerm for large throttle values
    float _TPA_multiplier {0.0F};
    float _TPA_breakpoint {0.6F};

    // by the time these are set, stick values have been converted to DPS
    float _rollStickDPS {0.0F};
    float _pitchStickDPS {0.0F};
    float _yawStickDPS {0.0F};
    float _maxRollRateDPS {500.0F};
    float _maxPitchRateDPS {500.0F};
    // angle mode data
    enum { CALCULATE_ROLL, CALCULATE_PITCH };
    uint32_t _angleModeCalculate { CALCULATE_ROLL };
    uint32_t _angleModeUseQuaternionSpace {false};
    float _rollStickDegrees {0.0F}; // roll stick in degrees
    float _rollStickSinAngle {0.0F};
    float _rollRateSetpointDPS {0.0F};
    float _rollSinAngle {0.0F};
    float _pitchStickDegrees {0.0F}; // pitch stick in degrees
    float _pitchStickSinAngle {0.0F};
    float _pitchRateSetpointDPS {0.0F};
    float _pitchSinAngle {0.0F};

    std::array<PIDF, PID_COUNT> _PIDS {};
    std::array<float, PID_COUNT> _outputs {}; //<! PID outputs. These are stored since the output from one PID may be used as the input to another
    const std::array<PIDF::PIDF_t, PID_COUNT> _scaleFactors;
    float _rollRateAtMaxPowerDPS {1000.0};
    float _pitchRateAtMaxPowerDPS {1000.0};
    float _yawRateAtMaxPowerDPS {1000.0};
    float _thrustOutput {0.0F};
    uint32_t _yawSpinRecovery { false };
    float _yawSpinRecoveredRPS { 100.0F * degreesToRadians };
    float _yawSpinPartiallyRecoveredRPS { 400.F * degreesToRadians };
    uint32_t _crashRecovery { false };

    filters_t _filters {};
    PowerTransferFilter1  _rollRateDTermFilter {};
    PowerTransferFilter1  _pitchRateDTermFilter {};
    PowerTransferFilter1  _rollStickFilter {};
    PowerTransferFilter1  _pitchStickFilter {};
};
