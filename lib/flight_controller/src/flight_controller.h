#pragma once

#include "flight_controller_telemetry.h"
#include "targets.h"

#include <filters.h>
#include <motor_commands.h>
#include <vehicle_controller_base.h>

#include <quaternion.h>
#include <string>

class Debug;
class DynamicNotchFilter;
class MotorMixerBase;

struct flight_controller_filters_config_t {
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

struct tpa_config_t {
    uint8_t tpa_mode;
    uint8_t tpa_rate;
    uint16_t tpa_breakpoint;
    int8_t tpa_low_rate;
    uint8_t tpa_low_always;
    uint16_t tpa_low_breakpoint;
};

struct anti_gravity_config_t {
    uint8_t cutoff_hz;
    uint8_t p_gain;
    uint8_t i_gain;
};

struct crash_flip_config_t {
    uint8_t motor_percent;
    uint8_t rate;
    uint8_t auto_rearm;
};

struct yaw_spin_recovery_config_t {
    int16_t yaw_spin_threshold;
    uint8_t yaw_spin_recovery;
};

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

struct iterm_relax_config_t {
    uint8_t iterm_relax_type; // not used
    uint8_t iterm_relax;        // Enable iterm suppression during stick input
    uint8_t iterm_relax_setpoint_threshold; // Full iterm suppression once setpoint has exceeded this value (degrees per second)
    uint8_t iterm_relax_cutoff; // Cutoff frequency used by low pass filter which predicts average response of the quad to setpoint
};

struct dmax_config_t {
    std::array<uint8_t, 2> dmax; // Maximum D value on each axis
    uint8_t dmax_gain;     // gain factor for amount of gyro / setpoint activity required to boost D
    uint8_t dmax_advance;  // percentage multiplier for setpoint
};

struct simplified_pid_settings_t {
    uint16_t multiplier;
    uint16_t roll_pitch_ratio;
    uint16_t i_gain;
    uint16_t d_gain;
    uint16_t pi_gain;
    uint16_t pitch_pi_gain;
    uint16_t dmax_gain;
    uint16_t k_gain;
};

enum fc_control_mode_e {
    FC_CONTROL_MODE_RATE = 0,
    FC_CONTROL_MODE_ANGLE = 1,
    FC_CONTROL_MODE_LEVEL_RACE = 2,
};

struct fc_controls_t {
    uint32_t tick_count;
    float throttle_stick;
    float roll_stick_dps;
    float pitch_stick_dps;
    float yaw_stick_dps;
    float roll_stick_degrees;
    float pitch_stick_degrees;
    fc_control_mode_e control_mode;
};


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
    FlightController(uint32_t task_interval_microseconds);
private:
    // FlightController is not copyable or moveable
    FlightController(const FlightController&) = delete;
    FlightController& operator=(const FlightController&) = delete;
    FlightController(FlightController&&) = delete;
    FlightController& operator=(FlightController&&) = delete;
public:
    static constexpr float RADIANS_TO_DEGREES = static_cast<float>(180.0 / 3.14159265358979323846);
    enum failsafe_e { FAILSAFE_OFF, FAILSAFE_ON };
    static constexpr int TIME_CHECKS_COUNT = 4;
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
    enum tpa_mode_e { TPA_MODE_PD, TPA_MODE_D, TPA_MODE_PDS };
    struct tpa_runtime_t {
        float breakpoint;
        float multiplier;
        float lowBreakpoint; //!!TODO: not used
        float lowMultiplier; //!!TODO: not used
    };
    static constexpr float DMAX_DEFAULT_ROLL = 40.0F;
    static constexpr float DMAX_DEFAULT_PITCH = 46.0F;
    static constexpr float DMAX_GAIN_FACTOR = 0.00008F;
    static constexpr float DMAX_SETPOINT_GAIN_FACTOR = 0.00008F;
    static constexpr float DMAX_RANGE_HZ = 85.0F;
    static constexpr float DMAX_LOWPASS_HZ = 35.0F;
    struct dmax_runtime_t {
        float gyroGain;
        float setpointGain;
        std::array<float, RP_AXIS_COUNT> percent;
        std::array<uint8_t, RP_AXIS_COUNT> max;
    };
    struct anti_gravity_runtime_t {
        float PGain;
        float IGain;
    };
#if defined(USE_YAW_SPIN_RECOVERY)
    enum yaw_spin_recovery_mode_e { YAW_SPIN_RECOVERY_OFF = 0, YAW_SPIN_RECOVERY_ON, YAW_SPIN_RECOVERY_AUTO };
    struct yaw_spin_recovery_runtime_t {
        float recovered_rps;
        float partially_recovered_rps;
    };
#endif
#if defined(USE_CRASH_RECOVERY)
    enum crash_recovery_e { CRASH_RECOVERY_OFF = 0, CRASH_RECOVERY_ON, CRASH_RECOVERY_BEEP, CRASH_RECOVERY_DISARM };
    struct crash_recovery_runtime_t {
        uint32_t timeLimitUs;
        uint32_t timeDelayUs;
        int32_t recoveryAngleDeciDegrees;
        float recoveryRate;
        float gyro_threshold_dps;
        float Dterm_threshold_dpsPS; // degrees per second per second
        float setpoint_threshold_dps;
        float limitYaw;
    };
#endif
#if defined(USE_ITERM_RELAX)
    enum iterm_relax_e { ITERM_RELAX_OFF, ITERM_RELAX_ON };
    struct iterm_relax_runtime_t {
        float setpoint_threshold_dps;
    };
#endif

    enum pid_tuning_mode_e {
        PID_TUNING_STANDARD = 0,
        PID_TUNING_SIMPLIFIED_RP,
        PID_TUNING_SIMPLIFIED_RPY,
        PID_TUNING_MODE_COUNT
    };
    static constexpr uint16_t PID_GAIN_MAX = 250;
    static constexpr uint16_t K_GAIN_MAX = 1000;

public:
    static float apply_deadband(float value, float deadband) {
        return (std::fabs(value) < deadband) ? 0.0F : (value >= 0.0F) ? value - deadband : value + deadband;
    }

    void motors_switch_off(MotorMixerBase& motor_mixer);
    void motors_switch_on(MotorMixerBase& motor_mixer);
    virtual uint32_t get_output_power_time_microseconds() const override; //!!TODO: is this still needed?

    fc_control_mode_e get_control_mode() const { return _rxC.control_mode; }
    void set_control_mode(fc_control_mode_e control_mode);

    pid_tuning_mode_e get_pid_tuning_mode() const { return _pid_tuning_mode; }
    void set_pid_tuning_mode(pid_tuning_mode_e pid_tuning_mode);

    const std::string& get_pid_name(pid_index_e pid_index) const;

    inline const PidController& get_pid(pid_index_e pid_index) const { return _sh.PIDS[pid_index]; }

    pid_constants_uint16_t get_pid_constants(pid_index_e pid_index) const;
    void set_pid_constants(pid_index_e pid_index, const pid_constants_uint16_t& pid16);

    const simplified_pid_settings_t& get_simplified_pid_Settings() const;
    void set_simplified_pid_Settings(const simplified_pid_settings_t& simplified_pid_settings);

    virtual pid_constants_uint16_t get_pid_msp(size_t index) const override;
    void set_pid_p_msp(pid_index_e pid_index, uint16_t kp);
    void set_pid_pd_msp(pid_index_e pid_index, uint16_t kp); // Set P and change D to preserve P/D ratio
    void set_pid_i_msp(pid_index_e pid_index, uint16_t ki);
    void set_pid_d_msp(pid_index_e pid_index, uint16_t kd);
    void set_pid_s_msp(pid_index_e pid_index, uint16_t ks);
    void set_pid_k_msp(pid_index_e pid_index, uint16_t kk);

    pid_error_t get_pid_error(size_t index) const override;
    float get_pid_setpoint(size_t index) const override;

    inline float get_pid_setpoint(pid_index_e pid_index) const { return _sh.PIDS[pid_index].get_setpoint(); }
    void set_pid_setpoint(pid_index_e pid_index, float setpoint) { _sh.PIDS[pid_index].set_setpoint(setpoint); }

    void switch_pid_integration_on() { for (auto& pid : _sh.PIDS) { pid.switch_integration_on();} }
    void switch_pid_integration_off() { for (auto& pid : _sh.PIDS) { pid.switch_integration_off();} }

    // Functions to calculate roll, pitch, and yaw rates in the NED coordinate frame, converting from gyro_rps in the ENU coordinate frame
    // Note that for NED, roll is about the y-axis and pitch is about the x-axis.
    static inline float roll_rate_ned_dps(const xyz_t& gyro_enu_rps) { return gyro_enu_rps.y * RADIANS_TO_DEGREES; }
    static inline float pitch_rate_ned_dps(const xyz_t& gyro_enu_rps) { return gyro_enu_rps.x * RADIANS_TO_DEGREES; }
    static inline float yaw_rate_ned_dps(const xyz_t& gyro_enu_rps) { return -gyro_enu_rps.z * RADIANS_TO_DEGREES; }

    static inline float roll_sin_angle_ned(const Quaternion& orientation) { return orientation.sin_pitch_clipped(); } // sin(x-180) = -sin(x)
    static inline float roll_cos_angle_ned(const Quaternion& orientation) { return orientation.cos_pitch(); }
    static inline float roll_angle_degrees_ned(const Quaternion& orientation) { return orientation.calculate_pitch_degrees(); }

    static inline float pitch_sin_angle_ned(const Quaternion& orientation) { return orientation.sin_roll_clipped(); } // NOTE: this is cheaper to calculate than sinRoll
    static inline float pitch_cos_angle_ned(const Quaternion& orientation)  { return orientation.cos_roll(); }
    static inline float pitch_angle_degrees_ned(const Quaternion& orientation) { return orientation.calculate_roll_degrees(); };

    static inline float yaw_angle_degrees_ned(const Quaternion& orientation) { return -orientation.calculate_yaw_degrees(); };

    flight_controller_quadcopter_telemetry_t get_telemetry_data(const MotorMixerBase& motor_mixer) const;

    inline uint32_t get_time_checks_microseconds(size_t index) const { return _sh.time_checks_microseconds[index]; } //!< Instrumentation time checks

    void set_max_angle_rates(float max_roll_rate_dps, float max_pitch_rate_dps, float maxYaw_rate_dps);
    float get_max_roll_angle_degrees() const { return _max_roll_angle_degrees; }
    float get_max_pitch_angle_degrees() const { return _max_pitch_angle_degrees; }

    void set_filters_config(const flight_controller_filters_config_t& filters_config);
    const flight_controller_filters_config_t& get_filters_config() const { return _filters_config; }

    void set_flight_mode_config(const flight_mode_config_t& flight_mode_config);
    const flight_mode_config_t& get_flight_mode_config() const { return _flight_mode_config; }

    void set_tpa_config(const tpa_config_t& tpa_config);
    const tpa_config_t& get_tpa_config() const { return _tpa_config; }

    void set_anti_gravity_config(const anti_gravity_config_t& anti_gravity_config);
    const anti_gravity_config_t& get_anti_gravity_config() const { return _anti_gravity_config; }

    void set_crash_flip_config(const crash_flip_config_t& crash_flip_config);
    const crash_flip_config_t& get_crash_flip_config() const { return _crash_flip_config; }

#if defined(USE_DMAX)
    void set_dmax_config(const dmax_config_t& dmax_config);
    const dmax_config_t& get_dmax_config() const { return _dmax_config; }
#endif

#if defined(USE_ITERM_RELAX)
    void set_iterm_relax_config(const iterm_relax_config_t& iterm_relax_config);
    const iterm_relax_config_t& get_iterm_relax_config() const { return _iterm_relax_config; }
#endif

#if defined(USE_YAW_SPIN_RECOVERY)
    void set_yaw_spin_recovery_config(const yaw_spin_recovery_config_t& yaw_spin_recovery_config);
    const yaw_spin_recovery_config_t& get_yaw_spin_recovery_config() const { return _yaw_spin_recovery_config; }
#endif

#if defined(USE_CRASH_RECOVERY)
    void set_crash_recovery_config(const crash_recovery_config_t& crash_recovery_config);
    const crash_recovery_config_t& get_crash_recovery_config() const { return _crash_recovery_config; }
#endif

public:
    [[noreturn]] static void Task(void* arg);
public:
    void detect_crash_or_spin();
    motor_commands_t apply_crash_flip_to_motors(const xyz_t& gyro_rps, float delta_t);
    void set_yaw_spin_threshold_dps(float yawSspin_threshold_dps);
    motor_commands_t recover_from_yaw_spin(const xyz_t& gyro_rps, float delta_t);

    void calculate_dmax_multipliers(Debug& debug);
    void initialize_setpoint_filters(float setpoint_delta_t);
    void apply_dynamic_pid_adjustments_on_throttle_change(float throttle, uint32_t tick_count, Debug& debug);
    void clear_dynamic_pid_adjustments();
    void update_setpoints(const fc_controls_t& controls, failsafe_e failsafe, Debug& debug);
    void update_rate_setpoints_for_angle_mode(const Quaternion& orientationENU, float delta_t);

    float calculate_iterm_error(size_t axis, float measurement, Debug& debug);
    virtual motor_commands_t calculate_motor_commands(const xyz_t& gyro_rps, const Quaternion& orientation, float delta_t, Debug& debug) override;

private:
    static constexpr float DEGREES_TO_RADIANS = 3.14159265358979323846F / 180.0F;
    DynamicNotchFilter* _dynamic_notch_filter {nullptr};

    //!!TODO: some constants below need to be made configurable
    const bool _use_quaternion_space_for_angle_mode {false};
    // ground mode handling
    const float _take_off_throttle_threshold {0.2F};
    const uint32_t _take_off_tick_threshold {1000};
    // other constants
    float _max_roll_rate_dps {500.0F};
    float _max_pitch_rate_dps {500.0F};
    float _maxYaw_rate_dps {500.0F};
    const float _max_roll_angle_degrees { 60.0F }; // used for angle mode
    const float _max_pitch_angle_degrees { 60.0F }; // used for angle mode

    //
    // configuration and runtime data is const once it has been set in set*Config()
    //
    pid_tuning_mode_e _pid_tuning_mode {PID_TUNING_STANDARD};
    flight_controller_filters_config_t _filters_config {};
    flight_mode_config_t _flight_mode_config {};
    tpa_config_t _tpa_config {};
    tpa_runtime_t _tpa { 0.0F, 1.0F, 0.0F, 1.0F };
    anti_gravity_config_t _anti_gravity_config {};
    anti_gravity_runtime_t _anti_gravity {};
    crash_flip_config_t _crash_flip_config {};
#if defined(USE_DMAX)
    dmax_config_t _dmax_config {};
    dmax_runtime_t _dmax {};
#endif
#if defined(USE_ITERM_RELAX)
    iterm_relax_config_t _iterm_relax_config = {.iterm_relax_type = 0, .iterm_relax=ITERM_RELAX_ON, .iterm_relax_setpoint_threshold = 40, .iterm_relax_cutoff=15};
    iterm_relax_runtime_t _iterm_relax = { .setpoint_threshold_dps = 40.0F };
#endif
#if defined(USE_YAW_SPIN_RECOVERY)
    yaw_spin_recovery_config_t _yaw_spin_recovery_config {.yaw_spin_threshold=1950, .yaw_spin_recovery=YAW_SPIN_RECOVERY_OFF};
    yaw_spin_recovery_runtime_t _yaw_spin = {.recovered_rps=100.0F*DEGREES_TO_RADIANS,. partially_recovered_rps=400.F*DEGREES_TO_RADIANS};
#endif
#if defined(USE_CRASH_RECOVERY)
    crash_recovery_config_t _crash_recovery_config {};
    crash_recovery_runtime_t _crash {};
#endif

    //
    // member data is divided into structs, according to which task may set that data
    // so, for example, only functions running in the context of the receiver task can set data using _rxM
    // this is to help avoid race conditions
    // data that can be set by more than one task is in the shared_t struct
    //
    struct fc_t {
        std::array<pid_constants_t, PID_COUNT> pid_constants {}; //!< the PID constants as set by tuning
        simplified_pid_settings_t simplified_pid_settings {};
    };
    struct rx_t {
        fc_control_mode_e control_mode {FC_CONTROL_MODE_RATE};
        // setpoint timing
        uint32_t setpointTickCount_previous {0};
        uint32_t setpointTickCountSum {0};
        enum { SETPOINT_TICKCOUNT_COUNTER_START = 100};
        uint32_t setpointTickCountCounter {SETPOINT_TICKCOUNT_COUNTER_START};
        float setpoint_delta_t {};
        float throttle_previous {0.0F};
        float yaw_rate_setpoint_dps {0.0F};
        uint32_t use_angle_mode {false}; // cache, to avoid complex condition test in update_outputs_using_pids
        uint32_t use_level_race_mode {false};
        float TPA {1.0F}; //!< Throttle PID Attenuation, reduces DTerm for large throttle values
#if defined(USE_ITERM_RELAX)
        std::array<float, RP_AXIS_COUNT> setpointLPs {};
        std::array<float, RP_AXIS_COUNT> setpointHPs {};
#endif
    }; // rx_t
    enum angle_mode_calculation_state_e { STATE_CALCULATE_ROLL, STATE_CALCULATE_PITCH };
    struct angle_mode_calculation_state_t {
        angle_mode_calculation_state_e state { STATE_CALCULATE_ROLL };
        float roll_sin_angle {0.0F};
        float pitch_sin_angle {0.0F};
    };
    struct ah_t {
        angle_mode_calculation_state_t amcs;
        std::array<float, RP_AXIS_COUNT> dmax_multiplier {1.0F, 1.0F}; // used even if USE_DMAX not defined
    }; // ah_t
    struct shared_t {
        uint32_t takeOffCountStart {0};
        bool ground_mode {true}; //! When in ground mode (ie pre-takeoff mode), the PID I-terms are set to zero to avoid integral windup on the ground
        bool crash_detected {false};
        bool crash_flip_mode_active {false};
        bool yaw_spin_recovery {false};
#if defined(USE_YAW_SPIN_RECOVERY)
        float yawSspin_threshold_dps {0.0F};
#endif
        float output_throttle {0.0F}; // throttle value is scaled to the range [-1,0, 1.0]
        std::array<PidController, PID_COUNT> PIDS {}; //!< PIDF controllers, with dynamically altered PID constants
        PowerTransferFilter2 anti_gravityThrottleFilter {};
        std::array<PowerTransferFilter1, PID_COUNT> dterm_filters1 {};
        std::array<PowerTransferFilter1, PID_COUNT> dterm_filters2 {};
        std::array<PowerTransferFilter3, RP_AXIS_COUNT> setpoint_derivative_filters {};
#if defined(USE_DMAX)
        std::array<PowerTransferFilter2, RP_AXIS_COUNT> dmaxRange_filters {};
        std::array<PowerTransferFilter2, RP_AXIS_COUNT> dmaxLowpass_filters {};
#endif
#if defined(USE_ITERM_RELAX)
        std::array<PowerTransferFilter1, RP_AXIS_COUNT> iterm_relax_filters {};
#endif
        std::array<PowerTransferFilter1, RPY_AXIS_COUNT> output_filters;
        // instrumentation data
        std::array<uint32_t, TIME_CHECKS_COUNT + 1> time_checks_microseconds {};
    }; // shared_t

    // Align data structures used in different tasks to avoid false sharing (https://en.wikipedia.org/wiki/False_sharing)
    enum { HARDWARE_DESTRUCTIVE_INTERFERENCE_SIZE = 64 };
    alignas(HARDWARE_DESTRUCTIVE_INTERFERENCE_SIZE) fc_t _fcM;          //!< MODIFIABLE partition of member data that CAN  be used in the context of the Flight Controller Task
    alignas(HARDWARE_DESTRUCTIVE_INTERFERENCE_SIZE) rx_t _rxM;          //!< MODIFIABLE partition of member data that CAN  be used in the context of the Receiver Task
    alignas(HARDWARE_DESTRUCTIVE_INTERFERENCE_SIZE) ah_t _ahM;          //!< MODIFIABLE partition of member data that CAN  be used in the context of the AHRS Task
    alignas(HARDWARE_DESTRUCTIVE_INTERFERENCE_SIZE) shared_t _sh;       //!< member data that is set in the context of more than one task
    alignas(HARDWARE_DESTRUCTIVE_INTERFERENCE_SIZE) const fc_t& _fcC = _fcM;   //!< CONSTANT   partition of member data that MUST be used outside the context of the Flight Controller Task
    const rx_t& _rxC = _rxM;   //!< CONSTANT   partition of member data that MUST be used outside the context of the Receiver Task

    // Betaflight compatible mixer output scale factor: scales roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
    static constexpr float MIXER_OUTPUT_SCALE_FACTOR = 0.001F;
    // Betaflight-compatible PID scale factors.
    static constexpr pid_constants_t _scale_factors = {
        0.032029F,
        0.244381F,
        0.000529F,
        0.01F, // !!TODO: provisional value
        0.013754F
    };
public:
    //!Default PIDs. For compatibility these are the same values as used by Betaflight.
    typedef const std::array<pid_constants_uint16_t, PID_COUNT> default_pids_t;
    static constexpr default_pids_t DEFAULT_PIDS = {{
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
