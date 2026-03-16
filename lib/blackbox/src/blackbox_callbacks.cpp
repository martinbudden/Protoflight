#include "blackbox_callbacks.h"
#include "cockpit.h"
#include "flight_controller.h"
#include "rc_modes.h"
#include <gps.h>
#include <gps_message_queue.h>


#include <ahrs.h>
#include <ahrs_message_queue.h>
#include <cmath>
#include <debug.h>
#if (__cplusplus >= 202002L)
#include <ranges>
#endif
#include <motor_mixer_base.h>
#include <receiver_base.h>


bool BlackboxCallbacks::is_armed(const blackbox_context_t& ctx) const
{
    return ctx.cockpit.is_armed();
}

bool BlackboxCallbacks::are_motors_running(const blackbox_context_t& ctx) const
{
    return ctx.motor_mixer.motors_is_on();
}

bool BlackboxCallbacks::is_blackbox_mode_active(const blackbox_context_t& ctx) const
{
    return ctx.rc_modes.is_mode_active(MspBox::BOX_BLACKBOX);
}

bool BlackboxCallbacks::is_blackbox_erase_mode_active(const blackbox_context_t& ctx) const
{
    return ctx.rc_modes.is_mode_active(MspBox::BOX_BLACKBOX_ERASE);
}

bool BlackboxCallbacks::is_blackbox_mode_activation_condition_present(const blackbox_context_t& ctx) const
{
    return ctx.rc_modes.is_mode_activation_condition_present(MspBox::BOX_BLACKBOX);
}

uint32_t BlackboxCallbacks::get_arming_beep_time_microseconds(const blackbox_context_t& ctx) const
{
    (void)ctx;
    return 0;
}

void BlackboxCallbacks::beep(const blackbox_context_t& ctx) const
{
    (void)ctx;
}

uint32_t BlackboxCallbacks::rc_mode_activation_mask(const blackbox_context_t& ctx) const
{
    return ctx.cockpit.get_flight_mode_flags();
}

void BlackboxCallbacks::load_slow_state(blackbox_slow_state_t& slow_state, const blackbox_context_t& ctx)
{
    //memcpy(&slow_state->flight_mode_flags, &_rc_mode_activation_bitset, sizeof(slow_state->flight_mode_flags)); //was flight_mode_flags;
    slow_state.flight_mode_flags = ctx.cockpit.get_flight_mode_flags();
    slow_state.state_flags = 0; // this is GPS state
    slow_state.failsafe_phase = ctx.cockpit.get_failsafe_phase();
    //slow_state.rxSignalReceived = ctx.receiver.isRxReceivingSignal();
    slow_state.rx_signal_received = (slow_state.failsafe_phase == Cockpit::FAILSAFE_IDLE);
    slow_state.rx_flight_channel_is_valid = (slow_state.failsafe_phase == Cockpit::FAILSAFE_IDLE);
}

/*!
Called from within Blackbox::logIteration().
*/
void BlackboxCallbacks::load_main_state(blackbox_main_state_t& main_state, uint32_t current_time_us, const blackbox_context_t& ctx)
{
    (void)current_time_us;

    const ahrs_data_t ahrs_data = ctx.ahrs_message_queue.get_received_ahrs_data();
    Quaternion orientation = ahrs_data.orientation;

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

    static constexpr float RADIANS_TO_DEGREES {180.0F / 3.14159265358979323846F};
    static constexpr float GYRO_SCALE {RADIANS_TO_DEGREES * 10.0F};

    main_state.gyro_adc[0] = static_cast<int16_t>(std::lroundf(ahrs_data.acc_gyro_rps.gyro_rps.x * GYRO_SCALE));
    main_state.gyro_adc[1] = static_cast<int16_t>(std::lroundf(ahrs_data.acc_gyro_rps.gyro_rps.y * GYRO_SCALE));
    main_state.gyro_adc[2] = static_cast<int16_t>(std::lroundf(ahrs_data.acc_gyro_rps.gyro_rps.z * GYRO_SCALE));
    main_state.gyro_unfiltered[0] = static_cast<int16_t>(std::lroundf(ahrs_data.gyro_rps_unfiltered.x * GYRO_SCALE));
    main_state.gyro_unfiltered[1] = static_cast<int16_t>(std::lroundf(ahrs_data.gyro_rps_unfiltered.y * GYRO_SCALE));
    main_state.gyro_unfiltered[2] = static_cast<int16_t>(std::lroundf(ahrs_data.gyro_rps_unfiltered.z * GYRO_SCALE));
    // just truncate for acc
    main_state.acc_adc[0] = static_cast<int16_t>(ahrs_data.acc_gyro_rps.acc.x * 4096);
    main_state.acc_adc[1] = static_cast<int16_t>(ahrs_data.acc_gyro_rps.acc.y * 4096);
    main_state.acc_adc[2] = static_cast<int16_t>(ahrs_data.acc_gyro_rps.acc.z * 4096);

    if (orientation.get_w() < 0.0F) {
        // negate orientation if W is negative
        orientation = -orientation;
    }
    (void)orientation;
    //main_state.orientation[0] = static_cast<int16_t>(orientation.getX() * 0x7FFF);
    //main_state.orientation[1] = static_cast<int16_t>(orientation.getY() * 0x7FFF);
    //main_state.orientation[2] = static_cast<int16_t>(orientation.getZ() * 0x7FFF);

    // iterate through roll, pitch, and yaw PIDs
#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{blackbox_main_state_t::XYZ_AXIS_COUNT})) {
#else
    for (size_t ii = 0; ii < blackbox_main_state_t::XYZ_AXIS_COUNT; ++ii) {
#endif
        const auto pid_index = static_cast<FlightController::pid_index_e>(ii);
        const PidController& pid = ctx.flight_controller.get_pid(pid_index);
        const pid_error_t pid_error = pid.get_error();
        main_state.axis_pid_p[ii] = static_cast<int32_t>(std::lroundf(pid_error.p));
        main_state.axis_pid_i[ii] = static_cast<int32_t>(std::lroundf(pid_error.i));
        main_state.axis_pid_d[ii] = static_cast<int32_t>(std::lroundf(pid_error.d));
        main_state.axis_pid_s[ii] = static_cast<int32_t>(std::lroundf(pid_error.s));
        main_state.axis_pid_k[ii] = static_cast<int32_t>(std::lroundf(pid_error.k));
        main_state.setpoint[ii] = static_cast<int16_t>(std::lroundf(pid.get_setpoint()));
#if defined(USE_MAGNETOMETER)
        main_state.mag_adc[ii] = static_cast<int16_t>(mag.magADC.v[ii]);
#endif
    }
    // log the final throttle value used in the mixer
    main_state.setpoint[3] = static_cast<int16_t>(std::lroundf(ctx.motor_mixer.get_throttle_command() * 1000.0F));

    // interval [1000,2000] for THROTTLE and [-500,+500] for ROLL/PITCH/YAW
    const receiver_controls_pwm_t controls = ctx.receiver.get_controls_pwm(); // returns controls in range [1000, 2000]
    main_state.rc_command[0] = static_cast<int16_t>(controls.roll - ReceiverBase::CHANNEL_MIDDLE);
    main_state.rc_command[1] = static_cast<int16_t>(controls.pitch - ReceiverBase::CHANNEL_MIDDLE);
    main_state.rc_command[2] = static_cast<int16_t>(controls.yaw - ReceiverBase::CHANNEL_MIDDLE);
    main_state.rc_command[3] = static_cast<int16_t>(controls.throttle);

    static_assert(static_cast<int>(blackbox_main_state_t::DEBUG_VALUE_COUNT) == static_cast<int>(Debug::VALUE_COUNT));
    main_state.debug = ctx.debug.getValues();

#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{ctx.motor_mixer.get_motor_count()})) {
#else
    for (size_t ii = 0; ii < ctx.motor_mixer.get_motor_count(); ++ ii) {
#endif
        main_state.motor[ii] = static_cast<int16_t>(std::lroundf(ctx.motor_mixer.get_motor_output(ii)));
#if defined(USE_DSHOT_TELEMETRY)
        main_state.erpm[ii] = static_cast<int16_t>(mixer.getMotorRPM(ii));
#endif
    }
    main_state.vbat_latest = static_cast<uint16_t>(11.6F * 10.0F);
    main_state.amperage_latest = static_cast<uint16_t>(0.67F * 10.0F);

#if defined(USE_BAROMETER)
    //main_state.baroAlt = baro.altitude;
#endif

#if defined(USE_RANGEFINDER)
    // Store the raw sonar value without applying tilt correction
    //main_state.surfaceRaw = rangefinderGetLatestAltitude();
#endif

    main_state.rssi = 0;//getRssi();

#if defined(USE_SERVOS)
    main_state.servo[0] = 1527;
    main_state.servo[1] = 1473;
    main_state.servo[2] = 1620;
    main_state.servo[4] = 1750;
    //for (size_t ii = 0; ii < blackboxMainState_t::MAX_SUPPORTED_SERVO_COUNT; ++ii) {
    //    main_state.servo[ii] = 1527;
    //}
#endif
// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}

void BlackboxCallbacks::load_gps_state(blackbox_gps_state_t& gps_state, const blackbox_context_t& ctx)
{
    if (!ctx.gps) {
        return;
    }
#if defined(USE_GPS)
    gps_message_data_t gps_message_data {};
    ctx.gps->getGPS_MessageQueue().PEEK_GPS_DATA(gps_message_data);

    gps_state.time_of_week_ms = gps_message_data.time_of_week_ms;
    //gps_state.interval_ms;
    //gps_state.homeLongitude_degrees1E7;
    //gps_state.homeLatitude_degrees1E7;
    //gps_state.homeAltitude_cm;
    gps_state.longitude_degrees1E7 = gps_message_data.longitude_degrees1E7;
    gps_state.latitude_degrees1E7 = gps_message_data.latitude_degrees1E7;
    gps_state.altitude_cm = gps_message_data.altitude_cm;
    gps_state.velocity_north_cmps = gps_message_data.velocity_north_cmps;
    gps_state.velocity_east_cmps = gps_message_data.velocity_east_cmps;
    gps_state.velocity_down_cmps = gps_message_data.velocity_down_cmps;
    gps_state.speed3d_cmps = gps_message_data.speed3d_cmps;
    gps_state.ground_speed_cmps = gps_message_data.ground_speed_cmps;
    gps_state.ground_course_deci_degrees= gps_message_data.heading_deci_degrees;
    gps_state.satellite_count = gps_message_data.satellite_count;
#else
    (void)gps_state;
#endif
}
